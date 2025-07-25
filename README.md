디렉터리 구조
=
mkdir -p xycar_ws/src/orda
cd xycar_ws/src/orda
git clone https://github.com/doldolmeng2/2025-kookmin-contest.git .



src
├── orda
|    ├── modualr
│    │    ├── main
│    │    ├── object-detection
│    │    ├── lane_detection
│    │    ├── traffic_light
│    │    ├── rubbercone
│    │    └── control
│    │
│    ├── monolithic
│    │    └── auto_drive
│    │
│    └── function
│         └── manual_drive
└── ...
