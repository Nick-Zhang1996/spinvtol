
Test result:
  long long ago
  5rev/s, 330g, 0.45m wingspan
  analysis c_l 0.95

  July 24, Wed
  added control system, weight 430g, full throttle barely takes of, never leaves heavy ground effect zone,  doesn't respond much to flap offset
  omega: 5rev/s (full throttle)
  wingspan: 0.45m, calculated c_l required to hover: 1.34 (max for clarky at Re=2e5 is 1.4)

  July 27, Sat
  Added extension to wing, now span is 0.8m, seems to have ample lift, but tips over easily. Made few modification to landing gear with little luck. 
  weight now 468g, omega = 2.5rev/s when tip over
  Analyzed c_l needed is 0.24 for 5rev/s
  for 2.5rev, c_l needed is 0.96
 
  July 28, Sun
  Reduced installation angle on the wing assembly (w/ extension), plane was able to take off rapidly, then throttle was retarded and plane starts to descent, as throttle is reapplied rapidly, wing seems to have a surge of lift and the monocopter tipped over.
  During later tests, prominent shudder from the ring-pipe landing gear halted the test. Angle of the ring was adjusted and tape was applied to reduce friction. Plane was not able to take off at 3.3rev/s (though it previously did)
  Maybe initial positioning of the aircraft is important?

  July 31, Wed
  Did multiple tests. The angle between the balancing weight(with batt and electronics) and the wing seem to have a large effect on the abnormal behavior of the monocopter. An anhedral would help the monocopter take off, though after that a rapid increase in disk angle would render the aircraft unusable

  It seems the motor's installation angle also plays a vital role here. If excessive noise is heard on the motor's landing gear, then it is likely that the aerodynamic moment is increasing, and countered by the landing gear's contact normal force with ground. If such restriction is removed by placing the monocopter on a stand, then the AOA of the wing is free to increase, and this will be an aggrevating, unstable increase that prevent steady hovering. Or, if the throttle input is chosen carefully, the increase in AOA will induce drag, therefore slowing the monocopter down, and the whole system enters a limit cycle of increase omega, increase AOA, increase drag, decrease omega, decrease moment, ...

  In previous successful flight with a light, no ctrl electronics and no ext wing monocopter, video examination shows an obvious downward pitching moment provided by the motor. However, the wing still assumes an AOA that allows positive lift. This seems to be absent in some of the test flights today. The aircraft either happily takes off then rapidly increase AOA till everything goes wrong, or bites and grinds the ground, never getting enough (or any) lift to get off ground. Maybe a sensible action is to increase installation angle of the wing, or attach some device that mechanically constrains the wing's initial pitch angle to ensure positive lift is generated. 
