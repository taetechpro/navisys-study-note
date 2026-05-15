# `practice/` — Clean-Room 학습 코드

`src/` 의 reference 구현을 **보지 않고** 사용자가 직접 클린룸으로 다시 짜보는 학습 슬롯들. C++ 와 VIO/EKF 알고리즘을 동시에 익히는 것이 목적이며, 메인 빌드 (`lc_vio_core`) 에는 포함되지 않는다.

## 학습 모드

각 슬롯은 다음 모드로 진행한다:

1. **시범** — Claude 가 정답 코드를 한 번에 보여줌
2. **옮기기** — 사용자가 그대로 옮겨 적고 저장
3. **의미 풀이** — 줄별 의미를 표/비유로 풀이
4. **의문 받기** — 사용자가 궁금한 부분만 깊게
5. **다음 sub-stage 로 이동**

Stage 가 충분히 익혀지면 점진적으로 "빈칸 채우기" 모드로 전환.

## 슬롯 구조

각 슬롯 디렉토리는 다음 형태를 갖는다:

```
practice/<topic>_clean_room/
├── my_<module>.hpp          # 사용자가 직접 작성 (헤더부터)
├── my_<module>.cpp          # 사용자가 직접 작성 (구현)
└── (선택) main_test.cpp     # 단독 검증 빌드용
```

진행 상태와 sub-stage 정의는 `docs/Practice/session_progress.md` 에 둔다 (한 곳에서 모든 슬롯의 진행률을 본다).

## 현재 슬롯

| 슬롯 | 대상 reference | 상태 |
|---|---|---|
| `lidar_clean_room/` | `src/lidar/lidar_segmenter.cpp` | Stage A 완료 (헤더 84줄), Stage B 다음 |

## 예정 슬롯 (TC MSCKF 전환 후)

| 슬롯 | 대상 reference | 학습 포인트 |
|---|---|---|
| `plane_constraint_clean_room/` | `src/plane_constraint/` (예정) | pseudo-measurement 수식, Jacobian 유도 |
| `tc_msckf_clean_room/` | `src/msckf/` (예정) | sliding window, nullspace projection, chi-squared gate |

## 새 슬롯 추가 규칙

1. 폴더명: `<topic>_clean_room/` (snake_case, 항상 `_clean_room` 접미사)
2. 진행 정의: `docs/Practice/session_progress.md` 에 stage table 추가
3. 메인 빌드에 추가하지 말 것 (별도 main_test.cpp 로만 빌드)
4. reference 위치를 README 첫 줄에 명시
