nav2_params.yaml은 odom 및 scan topic을 지속적으로 main pi에서 받아오는지의 여부가 매우 중요.

AMCL -> odom + scan을 통해서 로봇의 현재 위치를 추정 가능하게 함.

navigate_w_replanning_and_recovery.xml 
-> 경로 재계획, 장애물 발견 시 회피 행동 등 네비게이션의 전체 알고리즘을 담당

