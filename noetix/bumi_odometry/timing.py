"""Offline visual/gyro read-delay qualification; never supplies an SDK source time.

Both CSVs contain stamp_ns,wx,wy,wz in BODY axes and rad/s. Visual rates must be
computed from original body poses, not EKF output (which already used gyro).
Use excited rotations, measured SI/axes and a calibration dataset distinct from
accuracy acceptance. Constant rate cannot identify latency and is refused.
"""
from bisect import bisect_left
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def estimate_offset(visual,imu):
    for rows in (visual,imu):
        if len(rows)<100 or rows[-1][0]-rows[0][0]<5:
            raise ValueError('at least 5 seconds and 100 samples required')
        if any(not all(math.isfinite(v) for v in [t,*w]) for t,w in rows):
            raise ValueError('nonfinite timing record')
        if any(b[0]<=a[0] or b[0]-a[0]>.15 for a,b in zip(rows,rows[1:])):
            raise ValueError('timing record must be monotonic without long gaps')
    mean=[sum(w[i] for _,w in visual)/len(visual) for i in range(3)]
    excitation=sum(sum((w[i]-mean[i])**2 for i in range(3)) for _,w in visual)/len(visual)
    if excitation<.03**2: raise ValueError('insufficient rotational excitation')
    times=[r[0] for r in imu]
    def cost(rows,lag):
        errors=[]
        for t,w in rows:
            x=t+lag;j=bisect_left(times,x)
            if not 0<j<len(imu): continue
            a,wa=imu[j-1];b,wb=imu[j];f=(x-a)/(b-a)
            errors.append(sum((w[i]-wa[i]-(wb[i]-wa[i])*f)**2 for i in range(3)))
        return sum(errors)/len(errors) if len(errors)>=50 else float('inf')
    lags=[i*.005 for i in range(-50,51)]
    scored=sorted((cost(visual,lag),lag) for lag in lags)
    error,lag=scored[0]
    if abs(lag)>=.25 or error>excitation*.2:
        raise ValueError('timing fit is poor or delay outside search range')
    competing=[v for v,l in scored if abs(l-lag)>=.04]
    if not competing or min(competing)<=error+excitation*.002:
        raise ValueError('timing offset is ambiguous')
    middle=len(visual)//2
    splits=[min((cost(rows,l),l) for l in lags)[1] for rows in (visual[:middle],visual[middle:])]
    uncertainty=max(.005,abs(splits[0]-splits[1]))
    if uncertainty>.05: raise ValueError('read delay is unstable across recording')
    return dict(offset_s=lag,uncertainty_s=uncertainty,residual_rms_rad_s=math.sqrt(error),
                duration_s=visual[-1][0]-visual[0][0],split_offsets_s=splits,
                source_timestamp_available=False)


def read_csv(path):
    with Path(path).open() as stream:
        return [(int(row['stamp_ns'])/1e9,[float(row[k]) for k in ('wx','wy','wz')]) for row in csv.DictReader(stream)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual-csv',required=True);parser.add_argument('--imu-csv',required=True)
    parser.add_argument('--data-source',choices=['measured','synthetic'],required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=estimate_offset(read_csv(args.visual_csv),read_csv(args.imu_csv))
    digest=hashlib.sha256(Path(args.visual_csv).read_bytes()+Path(args.imu_csv).read_bytes()).hexdigest()
    result.update(data_source=args.data_source,qualification_id=digest,
                  mode='host_read_qualified' if args.data_source=='measured' else 'synthetic_only',max_age_s=.25,
                  note='Read-delay fit; source age remains unknown. Field qualification required.')
    Path(args.output).write_text(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__': main()
