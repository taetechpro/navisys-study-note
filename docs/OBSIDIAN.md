# Obsidian + LLM Wiki 사용 가이드

본 `docs/` 폴더는 Obsidian Vault 로 바로 열 수 있게 정리되어 있다. 이 가이드는 **Obsidian 시작 + LLM 통합 + 컨벤션** 을 한 번에 묶은 입문서.

> **목적**
> - 산재된 인사이트 / 학습 노트 / 가이드를 **그래프** 로 연결해 망각 방지
> - LLM 플러그인이 vault 안 마크다운을 임베딩 → "이 RANSAC 문제 어떻게 풀었지?" 같은 질문을 vault 안 노트로 답하게 함
> - 졸업 연구 후반부 (TC MSCKF, paper drafting) 까지 같은 vault 재활용

---

## 1. Vault 시작 (3 분)

1. Obsidian 다운로드: https://obsidian.md (Windows installer)
2. 실행 → "Open folder as vault" → `D:\02_research\04_cpp_seg_msckf_vio\docs` 선택
3. Vault 가 열리면 좌측에 모든 .md 파일이 트리로 보임
4. **Graph view** (Ctrl+G) 로 wiki-link 연결 그래프 확인

기본 설정 권장:
- **Settings → Files & Links → New link format: "Shortest path when possible"**
- **Settings → Files & Links → Use [[Wikilinks]]: ON**
- **Settings → Editor → Show frontmatter: ON**

---

## 2. 추천 플러그인 (Community + LLM)

`Settings → Community plugins → Turn on community plugins` 활성 후 Browse:

### 필수 (LLM 안 써도 유용)
| 플러그인 | 역할 |
|---|---|
| **Dataview** | frontmatter 기반 동적 쿼리. "insight 폴더에서 #ekf 태그 노트만 표" 같은 거 가능 |
| **Templater** | 새 노트 만들 때 frontmatter 자동 삽입 |
| **Tag Wrangler** | 태그 일괄 이름 변경 / 머지 |

### LLM 통합 (3 옵션 중 택 1~2)
| 플러그인 | 장점 | 단점 |
|---|---|---|
| **Smart Connections** | 자동 노트 임베딩 → 의미 유사 노트 자동 제안. OpenAI / Local LLM 둘 다 지원. 사이드 패널에 "이 노트와 관련된 다른 노트" 실시간 표시. | 무료지만 OpenAI API key 필요 (또는 local) |
| **Copilot for Obsidian** | ChatGPT 스타일 채팅창에 vault 컨텍스트 자동 주입. RAG 내장. | OpenAI / Claude API key 필요 |
| **BMO Chatbot** | 로컬 LLM (Ollama, LM Studio) 연동 우선. 무료 / 오프라인. | UI 가 단순함 |

**연구실 컴퓨터에 Ollama 가 이미 설치되어 있음** (`PATH` 에 `Ollama/` 있음) → **BMO** 가 무료 + 오프라인이라 시작용으로 좋음. 본격 사용 시 Smart Connections + OpenAI 로 업그레이드.

### LaTeX 수식 / 그림
| 플러그인 | 역할 |
|---|---|
| **Excalidraw** | 다이어그램 작성 (현재 KaTeX/MathJax 는 기본 지원) |
| **Image Toolkit** | 이미지 확대 / 줌 |

---

## 3. Frontmatter 컨벤션

모든 노트 맨 위에 YAML frontmatter 박는다. Obsidian 이 검색 / 필터 / 그래프 색상에 활용.

### 기본 템플릿
```yaml
---
title: "노트의 한 줄 제목"
date: 2026-05-22                    # 작성일 (YYYY-MM-DD)
type: insight | practice | guide | manual | theory
tags: [vio, ekf, segmentation]      # 아래 표준 태그 참조
related:
  - "[[20260514_lc_ekf_cam_imu_fusion]]"   # 관련 노트 wiki-link
  - "[[Practice/stage_d_ransac]]"
status: draft | reviewed | archived
---
```

### `type` 의 5 가지 (이 vault 의 카테고리)
- `insight` — 날짜별 판단/실험 기록 (`docs/insight/`)
- `practice` — 클린룸 학습 노트 (`docs/Practice/`)
- `guide` — 실행 절차 (`docs/guides/`)
- `manual` — 긴 매뉴얼 (`docs/manuals/`)
- `theory` — 논문/방법론 정리 (`docs/theory/`)

### Templater 예시 (new note 자동 frontmatter)
`Settings → Templater → Template folder location: _templates`
`_templates/insight.md`:
```
---
title: "<% tp.file.title.replace(/^[\d_]+/, '') %>"
date: <% tp.date.now("YYYY-MM-DD") %>
type: insight
tags: []
related: []
status: draft
---

## 요약


## 결정 / 판단


## 코드 / 수식


## 직관


## 다음 단계
```

---

## 4. 태그 시스템 (표준)

플랫하게 유지 (계층 nested 태그는 피함 — 검색 비용 ↑):

### 도메인 태그
- `#vio` — Visual-Inertial Odometry 전반
- `#ekf` — EKF 수식, Jacobian, 필터 자체
- `#msckf` — Multi-State Constraint Kalman Filter
- `#lc-ekf` — Loosely-Coupled (본 repo baseline)
- `#tc-msckf` — Tightly-Coupled (전환 목표)
- `#stereo` — Stereo 카메라
- `#imu` — IMU
- `#lidar` — LiDAR

### 알고리즘/기법 태그
- `#ransac` — RANSAC 평면 fitting
- `#segmentation` — floor/wall 분류
- `#gravity-prior` — 중력 prior 활용
- `#plane-constraint` — 평면 제약 (MSCKF update)

### 상태 태그
- `#open-question` — 미해결 의문
- `#decision` — 핵심 결정 기록
- `#experiment` — 실험 결과
- `#literature` — 논문 정리

여러 개 동시 가능: `tags: [vio, ekf, ransac, segmentation]`

---

## 5. Wiki-link 사용

### 기본
```
[[20260514_lc_ekf_cam_imu_fusion]]              ← 같은 vault 안 노트
[[Practice/stage_d_ransac]]                      ← 하위 폴더 포함
[[20260514_lc_ekf_cam_imu_fusion|LC-EKF 융합]]   ← 별명 표시
```

### 임시 / 미작성 노트 (planted seed)
```
[[20260601_tc_msckf_update_jacobian]]
```
존재 안 하는 wiki-link 는 회색 표시 + Ctrl+클릭 시 자동 생성. **앞으로 쓸 노트의 자리** 를 미리 박아두는 패턴 (=planted seed).

### Backlink 활용
모든 노트 우측 패널 "Backlinks" → 이 노트를 참조한 다른 노트 자동 표시.

---

## 6. MOC (Map of Contents) 패턴

vault 가 커지면 폴더 구조보다 **MOC 노트** 가 진입점. 이 vault 의 MOC:

| MOC 노트 | 역할 |
|---|---|
| `[[README]]` | 마스터 인덱스. 모든 영역 진입점 |
| `[[insight/README]]` | insight 노트 시간순 인덱스 |
| `[[Practice/session_progress]]` | 클린룸 학습 진행 상태 |
| `[[guides/README]]` | 실행 가이드 인덱스 |

새 영역 만들 때마다 MOC 한 개씩 추가. README 가 자연스러운 MOC.

---

## 7. LLM 통합 워크플로우

### 패턴 A — Smart Connections (자동 추천)
1. 노트 작성 중 → 우측 사이드바에 "Smart View" 자동 활성
2. 임베딩 기반으로 의미 유사 노트 자동 표시
3. "이거 전에 봤던 거 같은데?" 직감을 plugin 이 검증

### 패턴 B — Copilot 채팅 (질문→답변)
1. Cmd+J (또는 Ctrl+J) 로 채팅 열기
2. "Vault 안에서 LC vs TC MSCKF 비교 노트 찾아줘" 같은 질문
3. RAG 가 관련 노트 인용하며 답변

### 패턴 C — 노트 작성 보조
1. 새 insight 노트 시작 → 빈 섹션 ("## 결정 / 판단") 만 채움
2. LLM 에게 "이 결정의 근거를 vault 안 어느 노트와 연결할 수 있는지" 물음
3. 추천된 wiki-link 박기

### Local LLM (BMO + Ollama) 시작
```powershell
ollama pull llama3.1:8b
ollama serve
```
Obsidian BMO 설정에서:
- Provider: `ollama`
- URL: `http://localhost:11434`
- Model: `llama3.1:8b`

---

## 8. 청소 권고 (현재 vault 상태)

### 노이즈로 자리 차지하는 파일
- `docs/build/` — LaTeX 중간 산출물. .gitignore 처리됨. **삭제 권장**
- `docs/insight/.pdf_build_*/` — markdown→PDF 빌드 중간물. **삭제 권장**

### 청소 명령 (PowerShell, 사용자 확인 후 실행)
```powershell
Remove-Item -Recurse -Force "D:\02_research\04_cpp_seg_msckf_vio\docs\build"
Remove-Item -Recurse -Force "D:\02_research\04_cpp_seg_msckf_vio\docs\insight\.pdf_build_*"
```

(원본 .md / .tex / 최종 .pdf 는 그대로 유지. 재빌드 가능한 중간물만 제거.)

---

## 9. 다음 단계

1. Obsidian 설치 + vault 열기
2. 본 가이드 ↑ 권장 플러그인 1~2 개 (Dataview + BMO 권장 시작)
3. 새 insight 노트 작성 시 Templater 로 frontmatter 자동
4. 졸업 연구 TC MSCKF 진입 시 `[[20260601_tc_msckf_*]]` 식으로 planted seed 미리 박기

---

## 관련 노트
- [[README]] — vault 마스터 MOC
- [[insight/README]] — insight 영역 진입점
- [[LidarSegmenter session_progress]] — LidarSegmenter 클린룸 학습 (완료)
