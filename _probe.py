import sys, os
sys.path.insert(0, 'plane')
import spec
from parts import fuselage as fus, intake

print("station  fus(w,h,zc,n)         ductbore(w,h,zc,n)      slot_out_y  roof_z  skin_z_top")
for x in (115.0, 170.0, 195.0, 215.0, 220.0, 235.0, 258.0, 285.0, 290.0, 300.0):
    w, h, zc, n = fus.station_at(x)
    bw, bh, bzc, bn = intake.duct_bore(x)
    ow, oh, ozc = intake.duct_section(x)
    slot = w - spec.FUSELAGE_SKIN - ow   # free outboard slot half-width
    roof = intake.duct_top(x)
    print(f"{x:7.1f}  ({w:5.1f},{h:5.1f},{zc:6.1f},{n:.2f})  ({bw:5.1f},{bh:5.1f},{bzc:6.1f},{bn:.2f})  {slot:8.2f}  {roof:6.1f}  {zc+h:7.1f}")
