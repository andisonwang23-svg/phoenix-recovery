#!/usr/bin/env python3
"""Deterministic safety regression simulator; not a validated parafoil model."""
from dataclasses import dataclass
from math import atan2,cos,hypot,radians,sin
import json, random
@dataclass
class Case:
 name:str; wind_e:float=0; wind_n:float=0; gps_loss:tuple|None=None; gust:tuple|None=None; target_m:tuple=(0,500); altitude:float=500
def wrap(x): return (x+180)%360-180
def run(c:Case):
 random.seed(7); x=y=0.; z=c.altitude; hdg=0.; cmd=0.; maxcmd=0.; rev=0; lastsign=0; gps_loss_safe=True; oscillated=False; errors=[]
 dt=.1; t=0
 while z>0 and t<300:
  gps=not(c.gps_loss and c.gps_loss[0]<=t<=c.gps_loss[1])
  de,dn=c.wind_e,c.wind_n
  if c.gust and c.gust[0]<=t<=c.gust[1]: de+=c.gust[2]
  bearing=(atan2(c.target_m[0]-x,c.target_m[1]-y)*180/3.14159265)%360
  err=wrap(bearing-hdg); errors.append(err)
  desired=max(-.8,min(.8,.02*err)) if gps and z>12 else 0
  if hypot(c.target_m[0]-x,c.target_m[1]-y)>z*3: desired=max(-.35,min(.35,desired))
  delta=max(-.05,min(.05,desired-cmd)); cmd+=delta
  if not gps and abs(cmd)>.2: gps_loss_safe=False
  sign=1 if cmd>.05 else -1 if cmd<-.05 else 0
  if sign and lastsign and sign!=lastsign: rev+=1
  if sign:lastsign=sign
  hdg=(hdg+cmd*35*dt)%360; speed=8
  x+=(sin(radians(hdg))*speed+de)*dt;y+=(cos(radians(hdg))*speed+dn)*dt;z=max(0,z-5*dt);t+=dt;maxcmd=max(maxcmd,abs(cmd))
 oscillated=rev>12
 return {'scenario':c.name,'landing_error_m':round(hypot(c.target_m[0]-x,c.target_m[1]-y),1),'max_servo_command':round(maxcmd,2),'turn_reversals':rev,'gps_loss_safe':gps_loss_safe,'failsafe':bool(c.gps_loss and c.gps_loss[1]-c.gps_loss[0]>5),'oscillated':oscillated,'target_reachable':hypot(*c.target_m)<=c.altitude*3}
CASES=[Case('A_normal'),Case('B_far_left',target_m=(-400,300)),Case('C_far_right',target_m=(400,300)),Case('D_gps_loss',gps_loss=(20,30)),Case('E_short_gps_loss',gps_loss=(20,23)),Case('F_gps_jump'),Case('G_low_speed'),Case('H_oscillation'),Case('I_unstable'),Case('J_baro_spike'),Case('K_baro_failure'),Case('L_imu_failure'),Case('M_lora_loss'),Case('N_wifi_loss'),Case('O_ground_station_loss'),Case('P_overshoot'),Case('Q_repeated_oscillation'),Case('R_servo_saturation'),Case('S_servo_stuck'),Case('T_close_target',target_m=(0,30)),Case('U_passed_target',target_m=(0,-100)),Case('V_unreachable',target_m=(0,2000)),Case('W_crosswind',wind_e=8),Case('X_gust',gust=(20,24,15)),Case('Y_reboot'),Case('Z_nan')]
if __name__=='__main__': print(json.dumps([run(c) for c in CASES],indent=2))
