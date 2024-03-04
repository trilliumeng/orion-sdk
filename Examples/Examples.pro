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
    TrackSize \
    UserData

unix:SUBDIRS += \
    VideoPlayer
