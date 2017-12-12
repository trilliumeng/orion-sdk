TEMPLATE = subdirs

SUBDIRS += \
    EncodeDecode \
    GeoPoint \
    GpsAndHeading \
    LineOfSight \
    PathTrack \
    SendCommand \
    UserData

unix:SUBDIRS += \
    VideoPlayer
