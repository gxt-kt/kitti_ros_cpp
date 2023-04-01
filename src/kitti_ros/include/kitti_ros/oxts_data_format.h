#ifndef OXTS_DATA_FORMAT_H
#define OXTS_DATA_FORMAT_H

enum ImuDataFormat {
  lat = 0,
  lon,
  alt,
  roll,
  pitch,
  yaw,
  vn,
  ve,
  vf,
  vl,
  vu,
  ax,
  ay,
  az,
  af,
  al,
  au,
  wx,
  wy,
  wz,
  wf,
  wl,
  wu,
  pos_accuracy,
  vel_accuracy,
  navstat,
  numsats,
  posmode,
  velmode,
  orimode,
};

#endif
