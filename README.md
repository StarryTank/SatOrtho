# SatOrtho

**Satellite-Map-Prior-Assisted Real-Time Orthophoto Generation**

SatOrtho is a real-time UAV orthophoto generation framework for
GNSS-denied environments. It uses prior satellite maps for absolute
georeferencing, fuses map-derived poses with high-rate visual SLAM poses, and
incrementally updates the orthophoto during flight.

## Release status

This repository currently contains a partial source release. The following
components are intentionally excluded from this snapshot:

- the `gis_data` implementation (`include/gis_data/` and `src/gis_data/`);
- datasets, configuration files, generated results, and binary artifacts;
- build and evaluation scripts.

The complete implementation, datasets, and evaluation scripts will be made
public after the manuscript is accepted. Because required components are not
included yet, this snapshot is not a standalone buildable release.

## Included components

- `include/map/` and `src/map/`: map construction and pose-fusion interfaces
  and implementation;
- `include/utils/` and `src/utils/`: shared utility code;
- `include/zmq/` and `src/zmq/`: messaging support;
- `test/`: development and integration test programs.

## License

A license will be provided with the complete public release.
