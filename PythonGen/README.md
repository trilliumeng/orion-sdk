# Orion SDK - Python Bindings

Python bindings generator for Trillium Orion gimbal communication protocol.

## Automatic Generation

Python bindings are automatically generated when you run the main SDK build scripts:

```bash
# Linux/Mac
./GenerateOrionPublicPacket.sh

# Windows
GenerateOrionPublicPacketWin.bat
```

This generates Python bindings to `Communications/python/orion_sdk/`.

## Manual Generation

You can also generate bindings manually:

```bash
# From the Public directory
PYTHONPATH=./PythonGen python3 -m orion_sdk.cli \
    Communications/OrionPublicProtocol.xml \
    Communications/python/orion_sdk
```

## Using the Generated Bindings

```python
import sys
sys.path.insert(0, 'Communications/python')

from orion_sdk import OrionCmd, OrionConnection
from orion_sdk.enums import OrionMode_t

# Connect to gimbal via TCP
with OrionConnection.open_tcp('169.254.87.47') as conn:
    cmd = OrionCmd()
    cmd.Target = [0.5, 0.3]  # Pan, tilt in radians
    cmd.Mode = OrionMode_t.ORION_MODE_RATE
    conn.send(cmd)
```

## Validation

### Cross-Language Tests (C vs Python encoding)

After generating bindings (test harness is auto-compiled), run cross-language validation:

```bash
cd Communications/python/orion_sdk/tests
python3 test_cross_validation.py --harness ./test_harness -v
```

### CSV Comparison (C++ vs Python output)

Compare CSV output from telemetry recordings:

Using a skylink recording .ts file input into skylink's "Convert Replay to CSV" to obtain the cpp_Analysis output

```bash
cd PythonGen

# Generate CSV from telemetry file
python3 examples/orion_csv_generator.py recording.ts --output-dir ./output

# Compare with C++ generated CSV
python3 examples/compare_csv.py cpp_Analysis.csv python_Analysis.csv
python3 examples/compare_csv.py cpp_InsLog.csv python_InsLog.csv
```

**NOTE**: Many of the values will be close. The discrepancies that need to be investigated is gimbalAltMsl (cpp uses EGM96 convention python does not), payloadLDTemp, targetHeadingRad, targetHeadingStr, targetSpeedMps the way that those values may need to be updated in `orion_csv_generator.py`.

## Development

To install the generator as a package (optional, for development):

```bash
cd PythonGen
pip install -e ".[dev]"
pytest
```
