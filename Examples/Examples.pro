TEMPLATE = subdirs

SUBDIRS += \
    CameraInfo \
    EncodeDecode \
    GeoPoint \
    GpsAndHeading \
    LineOfSight \
    PathTrack \
    SendCommand \
    UserData

unix:SUBDIRS += \
    VideoPlayer
