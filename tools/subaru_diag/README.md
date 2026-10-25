# Subaru engage diagnostics

Read-only. Walks the most recent drive's rlogs on the device and prints a timeline of
cruise / sunnypilot / panda state, EPS and EyeSight fault bits, the camera's original
EyeSight messages, and what the C4 sends in their place.

```bash
cd /data/openpilot && PYTHONPATH=/data/openpilot python3 tools/subaru_diag/diag_engage.py      # latest drive
cd /data/openpilot && PYTHONPATH=/data/openpilot python3 tools/subaru_diag/diag_engage.py 1    # the drive before
```
