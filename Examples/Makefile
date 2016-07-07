DIRS = EncodeDecode LineOfSight GeoPoint GpsAndHeading PathTrack UserData

build:
	@for x in $(DIRS); do make -C $$x; done

clean:
	@for x in $(DIRS); do make -C $$x clean; done

