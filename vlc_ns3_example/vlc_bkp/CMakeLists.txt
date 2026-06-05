build_lib(
  LIBNAME vlc

  SOURCE_FILES
    model/vlc-net-device.cc
    model/vlc-rx-net-device.cc
    model/vlc-tx-net-device.cc
    model/vlc-mobility-model.cc
    model/vlc-error-model.cc
    model/vlc-snr.cc
    model/vlc-channel-model.cc
    model/vlc-propagation-loss-model.cc
    helper/vlc-channel-helper.cc
    helper/vlc-device-helper.cc

  HEADER_FILES
    model/vlc-net-device.h
    model/vlc-rx-net-device.h
    model/vlc-tx-net-device.h
    model/vlc-mobility-model.h
    model/vlc-error-model.h
    model/vlc-snr.h
    model/vlc-channel-model.h
    model/vlc-propagation-loss-model.h
    helper/vlc-channel-helper.h
    helper/vlc-device-helper.h

  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libpropagation}
    ${libpoint-to-point}
)
