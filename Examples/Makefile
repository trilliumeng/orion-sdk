DIRS = EncodeDecode LinuxComm LineOfSight GeoPoint GpsAndHeading UserData

build:
	@for x in $(DIRS); do make -C $$x; done

clean:
	@for x in $(DIRS); do make -C $$x clean; done

