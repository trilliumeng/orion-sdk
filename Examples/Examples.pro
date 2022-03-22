TEMPLATE = subdirs

SUBDIRS += \
    CameraInfo \
    EncodeDecode \
    GeoPoint \
    GpsAndHeading \
    LineOfSight \
    PathTrack \
    SendCommand \
    SendConfig \
    UserData

unix:SUBDIRS += \
    VideoPlayer
