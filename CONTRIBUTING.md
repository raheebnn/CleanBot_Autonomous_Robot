# Contributing to CleanBot

Thanks for your interest! Bug reports, hardware notes and pull requests are all welcome.

## Reporting issues

Please include:
- Which component (Pi sender, motor brain, PC receiver, SLAM receiver, calibration)
- OS and versions (Raspberry Pi OS, Ubuntu, Python, OpenCV, ORB-SLAM3 commit)
- Console output around the problem
- What you expected vs. what happened

## Pull requests

1. Fork the repo and create a branch from `main`.
2. Keep changes focused — one feature or fix per PR.
3. Make sure `python -m compileall raspberry_pi pc imu_calibration` passes.
4. **Never test motor changes with the wheels on the ground first.** Say in the PR how you tested.
5. Update the README or `docs/` if behaviour, ports or wiring change.

## Licensing

By contributing you agree your code is released under the MIT License. Do not add code copied from ORB-SLAM3 or other GPL projects into this repository.
