import argparse
import xml.etree.ElementTree as ET
from collections import defaultdict, OrderedDict
import matplotlib.pyplot as plt
import csv
import shutil
import re
from typing import Dict, List, Tuple, Iterable

# ====== 안전 파서/복구 ======
def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()

def _write(path: str, s: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(s)

def _backup(path: str) -> str:
    bak = path + ".bak"
    shutil.copy2(path, bak)
    return bak

def _try_parse(path: str):
    try:
        tree = ET.parse(path)
        root = tree.getroot()
        return True, tree, root, None
    except ET.ParseError as e:
        m = re.search(r"line (\d+), column (\d+)", str(e))
        line = int(m.group(1)) if m else None
        return False, None, None, line

def _fix_amp(s: str) -> str:
    # 이미 정상 엔티티는 보호하고 나머지 &만 &amp;로 치환
    s = s.replace("&lt;", "\uE000").replace("&gt;", "\uE001") \
         .replace("&amp;", "\uE002").replace("&quot;", "\uE003").replace("&apos;", "\uE004")
    s = s.replace("&", "&amp;")
    s = s.replace("\uE000", "&lt;").replace("\uE001", "&gt;") \
         .replace("\uE002", "&amp;").replace("\uE003", "&quot;").replace("\uE004", "&apos;")
    return s

def robust_repair(path: str) -> Tuple[bool, str]:
    """
    1) & 치환 → 파싱 시도
    2) 마지막 </timestep>까지 자르고 </emissions> 보장 → 시도
    3) 그래도 실패면 </timestep> 기준으로 뒤에서 앞으로 백트래킹(최대 50단계) → 시도
    성공 시 True와 백업 경로 반환, 실패 시 False.
    """
    original = _read(path)
    bak = _backup(path)

    # (A) & 치환
    s = _fix_amp(original)
    _write(path, s)
    ok, *_ = _try_parse(path)
    if ok:
        return True, bak

    # (B) 끝 잘라 닫기
    last_ts = s.rfind("</timestep>")
    if last_ts != -1:
        fixed = s[: last_ts + len("</timestep>")]
        if "</emissions>" not in fixed:
            fixed += "\n</emissions>\n"
        _write(path, fixed)
        ok, *_ = _try_parse(path)
        if ok:
            return True, bak

    # (C) 백트래킹(중간 파손 대응)
    positions = [m.start() for m in re.finditer(r"</timestep>", s)]
    if positions:
        tried = 0
        for pos in reversed(positions[:-1]):  # 마지막은 이미 시도
            tried += 1
            if tried > 50:
                break
            fixed = s[: pos + len("</timestep>")]
            if "</emissions>" not in fixed:
                fixed += "\n</emissions>\n"
            _write(path, fixed)
            ok, *_ = _try_parse(path)
            if ok:
                return True, bak

    # 실패 → 원본 복원
    _write(path, original)
    return False, bak

def safe_parse(path: str):
    ok, tree, root, err_line = _try_parse(path)
    if ok:
        return tree, root

    # 강화 복구 시도
    repaired, bak = robust_repair(path)
    if repaired:
        ok2, tree2, root2, _ = _try_parse(path)
        if ok2:
            return tree2, root2

    raise ET.ParseError(f"XML을 복구 시도했으나 여전히 파싱에 실패했습니다: {path} (마지막 오류 줄: {err_line})")


# ====== 핵심 파싱 ======
def parse_emission_xml_with_fuel(path: str):
    """
    emission.xml을 파싱하여
    - times: [t0, t1, ...]
    - veh_ids: set([...])
    - co2_ts: dict[veh_id] -> list of CO2(mg/s) aligned to times
    반환. 누락된 시점은 0으로 채움.
    """
    tree, root = safe_parse(path)

    times: List[float] = []
    for ts in root.findall("timestep"):
        t = float(ts.get("time"))
        times.append(t)
    if not times:
        raise ValueError(f"{path} 파일에서 <timestep> 태그를 찾을 수 없습니다.")
    times = sorted(list(OrderedDict.fromkeys(times)))

    veh_ids = set()
    co2_ts = defaultdict(lambda: [0.0] * len(times))
    fuel_ts = defaultdict(lambda: [0.0] * len(times))   # ml/s
    time_index = {t: i for i, t in enumerate(times)}

    for ts in root.findall("timestep"):
        t = float(ts.get("time"))
        idx = time_index[t]
        for veh in ts.findall("vehicle"):
            vid = veh.get("id")
            veh_ids.add(vid)
            c = veh.get("CO2")
            f = veh.get("fuel")
            co2_ts[vid][idx]  = float(c) if c is not None else 0.0
            fuel_ts[vid][idx] = float(f) if f is not None else 0.0

    return times, veh_ids, co2_ts, fuel_ts

def ensure_same_time_axis(times_off: List[float], times_on: List[float]):
    """
    두 실험의 time 축이 다르면, 공통 교집합 축 사용.
    """
    if times_off == times_on:
        return times_off, None, None
    set_off = set(times_off)
    set_on  = set(times_on)
    common = sorted(list(set_off & set_on))
    if len(common) < 2:
        raise ValueError("OFF/ON 실험의 시간축이 충분히 겹치지 않습니다. "
                 "SUMO 설정의 step-length나 seed 값을 동일하게 맞춰주세요.")
    idx_off = {t:i for i,t in enumerate(times_off)}
    idx_on  = {t:i for i,t in enumerate(times_on)}
    return common, idx_off, idx_on

def remap_series_to_common(times_src: List[float], series_dict: Dict[str, List[float]], common_times: List[float]):
    idx_src = {t:i for i,t in enumerate(times_src)}
    out: Dict[str, List[float]] = {}
    for vid, arr in series_dict.items():
        out[vid] = [arr[idx_src[t]] if t in idx_src else 0.0 for t in common_times]
    return out

# ====== 계산 유틸 ======
def integrate_rate_trapz(times: List[float], values_rate: List[float]) -> float:
    """
    Σ CO2(mg/s) * dt 적분 → mg 반환
    dt는 인접한 time 차이로 계산(가변 timestep 지원)
    """
    if len(times) != len(values_rate):
        raise ValueError("시간 배열과 값 배열의 길이가 일치하지 않습니다.")
    if len(times) < 2:
        return 0.0
    
    total = 0.0

    for i in range(1, len(times)):
        dt = times[i] - times[i-1]
        total += 0.5 * (values_rate[i-1] + values_rate[i]) * dt
    return total

def sum_all_vehicles_generic(times: List[float], veh_ids: Iterable[str], series_dict: Dict[str, List[float]]) -> Tuple[Dict[str, float], float]:
    per_vehicle = {}
    for vid in veh_ids:
        per_vehicle[vid] = integrate_rate_trapz(times, series_dict[vid])
    total = sum(per_vehicle.values())
    return per_vehicle, total


# ====== 메인 로직 ======
def main():
    ap = argparse.ArgumentParser(description="Compare CO₂ and Fuel (OFF vs ON) from SUMO emission XMLs.")
    ap.add_argument("--off", default="emission_out.xml", help="Emission XML for platooning OFF")
    ap.add_argument("--on",  default="emission_in.xml",  help="Emission XML for platooning ON")
    ap.add_argument("--out-csv", default="co2_fuel_summary.csv", help="Output summary CSV (per-vehicle + totals)")
    ap.add_argument("--out-ts-co2",  default="co2_timeseries.png",  help="Output CO₂ time-series PNG")
    ap.add_argument("--out-ts-fuel", default="fuel_timeseries.png", help="Output Fuel time-series PNG")
    ap.add_argument("--out-bar",     default="co2_fuel_totals.png", help="Output totals bar PNG")
    ap.add_argument("--out-ts-csv",  default=None, help="(Optional) Output CSV for aggregate time series")
    ap.add_argument("--only", nargs="*", default=None, help="Only include these vehicle IDs (exact match).")
    ap.add_argument("--only-re", default=None, help="Regex to filter vehicle IDs (applied after --only).")
    ap.add_argument("--tmin", type=float, default=60.0,
                    help="시작 시간 컷오프 (초). 이 시간 이후 데이터만 사용.")
    args = ap.parse_args()

    print(f"Reading OFF: {args.off}")
    times_off, veh_off, co2_off, fuel_off = parse_emission_xml_with_fuel(args.off)
    print(f"Reading ON : {args.on}")
    times_on,  veh_on,  co2_on,  fuel_on  = parse_emission_xml_with_fuel(args.on)

    # 타임축 정합
    common_times, idx_off, idx_on = ensure_same_time_axis(times_off, times_on)
    times = common_times
    if idx_off is not None:
        co2_off  = remap_series_to_common(times_off,  co2_off,  times)
        fuel_off = remap_series_to_common(times_off,  fuel_off, times)
        co2_on   = remap_series_to_common(times_on,   co2_on,   times)
        fuel_on  = remap_series_to_common(times_on,   fuel_on,  times)

    # 👇 여기부터 추가: t >= tmin 만 사용
    tmin = args.tmin
    if tmin > 0:
        keep_idx = [i for i, t in enumerate(times) if t >= tmin]
        # times 잘라내기
        times = [times[i] for i in keep_idx]

        def _slice_dict(d):
            for vid, seq in d.items():
                d[vid] = [seq[i] for i in keep_idx]

        _slice_dict(co2_off)
        _slice_dict(co2_on)
        _slice_dict(fuel_off)
        _slice_dict(fuel_on)

    # 차량 집합(합집합 → 선택 필터)
    veh_all = sorted(list(veh_off | veh_on))
    if args.only:
        only_set = set(args.only)
        veh_all = [v for v in veh_all if v in only_set] or veh_all
    if args.only_re:
        cre = re.compile(args.only_re)
        veh_all = [v for v in veh_all if cre.search(v)] or veh_all

    # 누락 차량 0 시계열로 채우기
    for vid in veh_all:
        co2_off.setdefault(vid,  [0.0]*len(times))
        co2_on.setdefault(vid,   [0.0]*len(times))
        fuel_off.setdefault(vid, [0.0]*len(times))
        fuel_on.setdefault(vid,  [0.0]*len(times))

    # 차량별 및 전체 누적
    perveh_off_co2_mg, total_off_co2_mg = sum_all_vehicles_generic(times, veh_all, co2_off)
    perveh_on_co2_mg,  total_on_co2_mg  = sum_all_vehicles_generic(times, veh_all, co2_on)
    perveh_off_fuel_ml, total_off_fuel_ml = sum_all_vehicles_generic(times, veh_all, fuel_off)
    perveh_on_fuel_ml,  total_on_fuel_ml  = sum_all_vehicles_generic(times, veh_all, fuel_on)

    total_off_co2_g = total_off_co2_mg / 1000.0
    total_on_co2_g  = total_on_co2_mg  / 1000.0
    total_off_fuel_L = total_off_fuel_ml / 1000.0
    total_on_fuel_L  = total_on_fuel_ml  / 1000.0

    co2_reduction_pct  = (total_off_co2_g  - total_on_co2_g)  / total_off_co2_g  * 100.0 if total_off_co2_g  > 0 else 0.0
    fuel_reduction_pct = (total_off_fuel_L - total_on_fuel_L) / total_off_fuel_L * 100.0 if total_off_fuel_L > 0 else 0.0

    print(f"[OFF] CO2:  {total_off_co2_g:.3f} g | Fuel: {total_off_fuel_L:.3f} L")
    print(f"[ON ] CO2:  {total_on_co2_g:.3f} g | Fuel: {total_on_fuel_L:.3f} L")
    print(f"Reduction  CO2: {co2_reduction_pct:.2f}% | Fuel: {fuel_reduction_pct:.2f}%")

    # === CSV 저장 ===
    with open(args.out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["vehicle_id",
                    "total_CO2_OFF_g", "total_CO2_ON_g", "reduction_CO2_g", "reduction_CO2_pct",
                    "total_FUEL_OFF_L", "total_FUEL_ON_L", "reduction_FUEL_L", "reduction_FUEL_pct"])
        for vid in veh_all:
            off_co2_g = perveh_off_co2_mg[vid] / 1000.0
            on_co2_g  = perveh_on_co2_mg[vid]  / 1000.0
            red_co2_g = off_co2_g - on_co2_g
            red_co2_pct = (red_co2_g/off_co2_g*100.0) if off_co2_g > 0 else 0.0

            off_fuel_L = perveh_off_fuel_ml[vid] / 1000.0
            on_fuel_L  = perveh_on_fuel_ml[vid]  / 1000.0
            red_fuel_L = off_fuel_L - on_fuel_L
            red_fuel_pct = (red_fuel_L/off_fuel_L*100.0) if off_fuel_L > 0 else 0.0

            w.writerow([vid,
                        f"{off_co2_g:.6f}", f"{on_co2_g:.6f}", f"{red_co2_g:.6f}", f"{red_co2_pct:.2f}",
                        f"{off_fuel_L:.6f}", f"{on_fuel_L:.6f}", f"{red_fuel_L:.6f}", f"{red_fuel_pct:.2f}"])
        w.writerow([])
        w.writerow(["__TOTAL__",
                    f"{total_off_co2_g:.6f}", f"{total_on_co2_g:.6f}", f"{(total_off_co2_g-total_on_co2_g):.6f}", f"{co2_reduction_pct:.2f}",
                    f"{total_off_fuel_L:.6f}", f"{total_on_fuel_L:.6f}", f"{(total_off_fuel_L-total_on_fuel_L):.6f}", f"{fuel_reduction_pct:.2f}"])
    print(f"Saved: {args.out_csv}")


    # === 그래프 (시간추세: 전체 합) ===
    total_ts_off_co2 = [0.0]*len(times)
    total_ts_on_co2  = [0.0]*len(times)
    total_ts_off_fuel = [0.0]*len(times)
    total_ts_on_fuel  = [0.0]*len(times)

    for vid in veh_all:
        arr_off = co2_off[vid]
        arr_on  = co2_on[vid]
        total_ts_off_co2 = [a+b for a,b in zip(total_ts_off_co2, arr_off)]
        total_ts_on_co2  = [a+b for a,b in zip(total_ts_on_co2,  arr_on)]

        arr_off_fuel = fuel_off[vid]
        arr_on_fuel  = fuel_on[vid]
        total_ts_off_fuel = [a+b for a,b in zip(total_ts_off_fuel, arr_off_fuel)]
        total_ts_on_fuel  = [a+b for a,b in zip(total_ts_on_fuel,  arr_on_fuel)]

    if args.out_ts_csv:
        with open(args.out_ts_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s",
                        "sum_CO2_OFF_mg_s", "sum_CO2_ON_mg_s",
                        "sum_FUEL_OFF_ml_s", "sum_FUEL_ON_ml_s"])
            for t, c_off, c_on, fu_off, fu_on in zip(times, total_ts_off_co2, total_ts_on_co2,
                                                      total_ts_off_fuel, total_ts_on_fuel):
                w.writerow([f"{t:.3f}", f"{c_off:.6f}", f"{c_on:.6f}", f"{fu_off:.6f}", f"{fu_on:.6f}"])
        print(f"Saved: {args.out_ts_csv}")

    plt.figure(figsize=(11,5))
    plt.plot(times, total_ts_off_co2, label="OFF (sum of selected vehicles)")
    plt.plot(times, total_ts_on_co2,  label="ON  (sum of selected vehicles)")
    plt.xlabel("Time (s)")
    plt.ylabel("CO₂ (mg/s)")
    plt.title("CO₂ time series (ON vs OFF)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_ts_co2, dpi=150)
    plt.close()
    print(f"Saved: {args.out_ts_co2}")

    # === 그래프 (총합 막대) ===
    plt.figure(figsize=(11,5))
    plt.plot(times, total_ts_off_fuel, label="OFF (sum of selected vehicles)")
    plt.plot(times, total_ts_on_fuel,  label="ON  (sum of selected vehicles)")
    plt.xlabel("Time (s)")
    plt.ylabel("Fuel (ml/s)")
    plt.title("Fuel time series (ON vs OFF)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out_ts_fuel, dpi=150)
    plt.close()
    print(f"Saved: {args.out_ts_fuel}")

    plt.figure(figsize=(7,5))
    labels = ["CO₂ OFF (g)", "CO₂ ON (g)", "Fuel OFF (L)", "Fuel ON (L)"]
    vals = [total_off_co2_g, total_on_co2_g, total_off_fuel_L, total_on_fuel_L]
    bars = plt.bar(labels, vals)
    for b in bars:
        h = b.get_height()
        plt.text(b.get_x()+b.get_width()/2, h, f"{h:.2f}", ha="center", va="bottom")
    plt.title(f"Totals (CO₂↓ {co2_reduction_pct:.2f}% | Fuel↓ {fuel_reduction_pct:.2f}%)")
    plt.ylabel("CO₂ (g) / Fuel (L)")
    plt.tight_layout()
    plt.savefig(args.out_bar, dpi=150)
    plt.close()
    print(f"Saved: {args.out_bar}")

if __name__ == "__main__":
    main()
