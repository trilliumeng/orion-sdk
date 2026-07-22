TEMPLATE = subdirs

SUBDIRS +=     CameraInfo     EncodeDecode     GeoPoint     GpsAndHeading     LineOfSight     MultipleConnections     PathTrack     SendCommand     SendConfig     TrackSize     UserData

win32:SUBDIRS +=     MotionControl

unix:SUBDIRS +=     VideoPlayer
