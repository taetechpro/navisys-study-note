# `references/` — 외부 참조 코드

> 이 폴더는 **읽기 전용 참고 코드** 자리. git 추적되지 않음 (`.gitignore` 처리).
> 본 프로젝트가 의존하는 코드는 `third_party/` (예정) 에 별도 둠.

## 클론 방법

처음 설정 시 한 번만 실행:

```bash
cd D:/02_research/04_cpp_LC-EKF_VIO/references
git clone https://github.com/rpng/ov_plane.git
```

WSL 에서 실행 시:
```bash
cd /mnt/d/02_research/04_cpp_LC-EKF_VIO/references
git clone https://github.com/rpng/ov_plane.git
```

## 현재 참고 대상

### `references/ov_plane/` — Phase 0 (5/18~5/22) 분석 대상

- 논문: Chen, Geneva et al., "Monocular Visual-Inertial Odometry with Planar Regularities", ICRA 2023
- GitHub: https://github.com/rpng/ov_plane
- 본 프로젝트 핵심 진입점:
  - `ov_msckf/src/update/UpdaterMSCKF.cpp` ← 5/18 첫 통독 대상
  - `ov_msckf/src/state/State.h` ← sliding window state 구조
  - `ov_core/src/track/TrackKLT.cpp` ← multi-frame feature track
  - `ov_msckf/src/update/UpdaterHelper.cpp` ← nullspace projection

## 결정 후 처리

Phase 0 종료 (5/22) 시:
- **Fork 결정** → `references/ov_plane/` → `third_party/ov_plane/` 로 이동, git submodule 화
- **직접 구현 결정** → `references/ov_plane/` 유지 (계속 참고용으로 둠)
