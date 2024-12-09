# path_planning

## path planning test
![path_planning_test_Astar](https://github.com/user-attachments/assets/a9cc7547-a035-4f4e-80b8-7412b3a3098b)

해당 알고리즘은 임의의 Way point를 기반으로 경로를 생성
* 임의의 Way point는 csv파일에서 원하는 위치의 이름 상자 값을 GPT를 통해 좌표로 변환
* Way point 선정 기준 = 탐색 가능 & 각 지점의 끝 점
* path planning 알고리즘 = A star(A*)
  
## Way point 추정 
![way_poin](https://github.com/user-attachments/assets/c05c3bab-109b-4229-b2c9-fdd86b95a22d)

해당 이미지는 


# 최종 결과
![way_point_path_plannig_result](https://github.com/user-attachments/assets/25a4d10b-11d3-40b6-9fee-025e98994c09)
<br/>
"""
# 거리 변환 및 A* 알고리즘을 이용한 최적 경로 탐색

## 개요

이 프로젝트는 로봇이 장애물(벽)과의 거리를 고려하여 안전하게 이동할 수 있는 경로를 계산하는 알고리즘을 구현합니다. 이 알고리즘은 **거리 변환**과 **조정된 A*** 접근 방식을 활용하여 최적 경로를 생성합니다.

---

## 1. 기본 원리

알고리즘은 다음 단계를 통해 경로를 생성합니다:

1. **거리 변환**
   `distance_transform_edt` 함수를 사용하여 각 그리드 점이 장애물에서 얼마나 떨어져 있는지를 계산합니다. 결과는 **거리 맵**으로, 값이 클수록 장애물에서 멀리 떨어져 있음을 나타냅니다.

2. **가중치 맵 생성**
   거리 맵을 반전하여 **가중치 맵**을 생성합니다. 가중치 맵에서는 장애물에서 멀어질수록 가중치가 낮아집니다. 이 맵은 A* 알고리즘에서 사용되어 장애물과 거리를 유지하는 경로를 탐색합니다.

3. **A* 알고리즘**
   A* 알고리즘은 휴리스틱(유클리드 거리)과 총 비용을 결합하여 최적 경로를 탐색합니다. 이 과정에서 가중치 맵을 통합하여 장애물에서 멀리 떨어진 경로를 우선적으로 찾습니다.

---

## 2. 상세 계산

### (1) 거리 변환

`distance_transform_edt`를 사용하여 거리 맵을 계산합니다.

- **입력:** `grid_array` (0: 벽, 1: 경로)  
- **출력:** `distance_map`

수식으로 표현하면:

\(
d(i, j) = \min_{(x, y) \in \text{walls}} \sqrt{(i - x)^2 + (j - y)^2}
\)

여기서 \( d(i, j) \)는 좌표 \((i, j)\)가 가장 가까운 벽까지의 최소 거리입니다.

---

### (2) 가중치 맵

가중치 맵은 거리 맵을 반전하여 생성됩니다. 장애물에서 멀수록 가중치가 낮아집니다.

- **입력:** `distance_map`  
- **출력:** `weighted_grid`

수식으로 표현하면:

\(
w(i, j) = D_{\max} - d(i, j)
\)

여기서:
- \( w(i, j) \): 좌표 \((i, j)\)의 가중치
- \( D_{\max} \): 거리 맵의 최대값

---

### (3) A* 알고리즘

A* 알고리즘은 다음 비용 함수를 사용하여 최적 경로를 탐색합니다:

\(
f(n) = g(n) + h(n)
\)

여기서:
- \( f(n) \): 노드 \( n \)의 총 비용
- \( g(n) \): 시작점에서 노드 \( n \)까지의 실제 비용 (가중치 맵 기반으로 계산)
- \( h(n) \): 노드 \( n \)에서 목표점까지의 휴리스틱 비용 (유클리드 거리)

휴리스틱은 다음과 같이 계산됩니다:

\(
h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}
\)

---

## 3. 경로 탐색 과정

1. **시작점 추가**
   시작 노드에 대해 \( f(n) = h(n) \)을 계산하고 우선순위 큐에 추가합니다.

2. **이웃 탐색**
   현재 노드 \( n \)의 네 방향(상, 하, 좌, 우)을 평가합니다. 이웃 노드의 \( g(n) \)을 계산하고 \( f(n) \)과 함께 우선순위 큐에 추가합니다.

3. **목표점 도달**
   목표 노드가 큐에서 꺼내질 때 탐색을 종료하고 최적 경로를 재구성합니다.

---

## 4. 시각화

- **경로 색상**  
  계산된 경로는 빨간색(시작점)에서 파란색(목표점)으로 점진적으로 변하는 색상으로 시각화됩니다.

- **웨이포인트**  
  웨이포인트는 하얀 원으로 표시되어 주요 지점을 나타냅니다.

---

## 5. 결론

이 방법은 로봇이 장애물과의 거리를 고려하여 가장 안전하고 효율적인 경로를 계산합니다. 벽에 가까운 경로는 높은 비용으로 인해 탐색에서 제외되며, 벽에서 멀리 떨어진 경로가 우선적으로 탐색됩니다.

거리 기반 가중치 맵과 A* 알고리즘을 통합함으로써, 이 접근법은 자율 시스템에 최적의 안전 내비게이션을 보장합니다.

---
