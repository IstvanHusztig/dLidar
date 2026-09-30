#!/usr/bin/env python3
"""Report ordering and rate of the timestamps in an imu*.csv."""
import sys, statistics as st
rows=[l.split() for l in open(sys.argv[1]).readlines()[1:]]
t=[int(float(r[7])) for r in rows]; u=[int(float(r[8])) for r in rows]
d=[t[i+1]-t[i] for i in range(len(t)-1)]
print("rows",len(t),"span s",(t[-1]-t[0])/1e9,"rate Hz",len(t)/((t[-1]-t[0])/1e9))
print("non-increasing timestamp:",sum(x<=0 for x in d),
      " timestampUnix:",sum(u[i+1]<=u[i] for i in range(len(u)-1)))
print("median dt ms",st.median(d)/1e6,"min",min(d)/1e6,"max",max(d)/1e6)
