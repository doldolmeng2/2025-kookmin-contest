# 2025 제8회 국민대학교 자율주행 경진대회

# 디렉터리 생성 및 클론
mkdir -p xycar_ws/src/orda  
cd xycar_ws/src/orda  
git clone https://github.com/doldolmeng2/2025-kookmin-contest.git .  

# 소스코드 파일 구조
```
src
├── orda
│    ├── modular
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
```


# 브랜치 구조
```
main
├── modular-main
│    ├── modular/main
│    ├── modular/object_detection
│    ├── modular/lane_detection
│    ├── modular/traffic_light
│    ├── modular/rubbercone
│    └── modular/control
└── monolithic-main // (예시)

```
