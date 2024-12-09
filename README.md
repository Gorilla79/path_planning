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

1. 기본 원리
이 코드는 다음 단계를 거쳐 경로를 생성합니다:

* 거리 변환 (Distance Transform):

  * distance_transform_edt를 사용하여 그리드 맵에서 벽(값이 0)로부터 각 점까지의 거리를 계산합니다.
  * 결과는 벽에서 멀수록 값이 커지는 거리 맵입니다.

* 가중치 맵 생성:

  * 거리 맵을 반전하여, 벽에서 멀수록 낮은 가중치를 갖도록 조정합니다.
  * A* 알고리즘에서 이 가중치 맵을 사용하여 벽과 거리를 유지하면서 경로를 찾습니다.

* _A 알고리즘:_*

  * 휴리스틱(유클리드 거리)과 현재 노드까지의 총 비용을 합산하여 경로를 탐색합니다.
  * 최소 비용 경로를 탐색하는 과정에서 가중치 맵을 활용하여 벽과의 거리를 유지합니다.


2. 세부 계산
(1) 거리 변환
  distance_transform_edt를 사용하여 거리 맵을 계산합니다.

* 입력: grid_array (0: 벽, 1: 경로)
* 출력: distance_map
  수식으로 표현하면:

  d(i,j)=min⁡(x,y)∈벽(i−x)2+(j−y)2d(i, j) = \min_{(x, y) \in \text{벽}} \sqrt{(i - x)^2 + (j - y)^2}d(i,j)=(x,y)∈wallsmin​(i−x)2+(j−y)2​

  여기서 d(i,j)d(i, j)d(i,j)는 좌표 (i,j)(i, j)(i,j)가 벽까지의 최소 거리입니다.

(2) 가중치 맵
거리 맵을 반전하여 벽에서 멀수록 낮은 가중치를 갖게 만듭니다.

입력: distance_map
출력: weighted_grid
수식:

w(i,j)=Dmax⁡−d(i,j)w(i, j) = D_{\max} - d(i, j)w(i,j)=Dmax​− d(i,j)

여기서 w(i,j)w(i, j)w(i,j)는 좌표 (i,j)(i, j)(i,j)의 가중치이고, Dmax⁡D_{\max}Dmax​는 거리 맵의 최대값입니다.

(3) A 알고리즘*
A* 알고리즘은 다음 비용 함수를 사용하여 최적 경로를 탐색합니다:

f(n)=g(n)+h(n)f(n) = g(n) + h(n)f(n)=g(n)+h(n)

f(n)f(n)f(n): 현재 노드 nnn의 총 비용
g(n)g(n)g(n): 시작점에서 현재 노드까지의 실제 비용 (가중치 맵을 기반으로 계산)
h(n)h(n)h(n): 현재 노드에서 목표점까지의 휴리스틱 비용 (유클리드 거리)
휴리스틱:

h(n)=(xn−xg)2+(yn−yg)2h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2 }h(n)=(xn​−xg​)2+(in​−yg​)2​


3. 경로 탐색 과정
시작점 추가:

f(n)=h(n)f(n) = h(n)f(n)=h(n)로 계산하여 우선순위 큐에 추가합니다.
이웃 탐색:

현재 노드 nnn의 4방향(상, 하, 좌, 우)을 탐색합니다.
이웃 노드의 비용 g(n)g(n)g(n)를 계산하고, f(n)f(n)f(n)로 우선순위 큐에 추가합니다.
목표점 도달:

목표점이 큐에서 꺼내질 때 탐색을 종료하고 경로를 재구성합니다.


4. 시각화
계산된 경로는 다양한 색상으로 표시됩니다.
빨강 → 파랑으로 진행하면서 경로 색상이 변합니다.
웨이포인트는 흰색 원으로 표시됩니다.


5. 결론
이 방법은 벽과의 거리를 고려하여 로봇이 안전하게 이동할 수 있는 최적 경로를 계산합니다. 벽과 가까운 경로는 높은 비용을 부여해 탐색에서 제외되며, 벽에서 멀리 떨어진 경로가 우선 탐색됩니다.
