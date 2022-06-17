# Weapon Attack Animation Synthesis using Deep Reinforcement Learning

[시연 동영상](https://youtu.be/ahxzhjmgX00)

[발표 자료](https://github.com/CritBear/RL_AttackAnimation/blob/main/%5B%EA%B9%80%EA%B2%BD%EB%AF%BC%5D%20%EC%BA%A1%EC%8A%A4%ED%86%A4%EB%94%94%EC%9E%90%EC%9D%B8%20%EC%B5%9C%EC%A2%85%EB%B0%9C%ED%91%9C%EC%9E%90%EB%A3%8C.pdf)

### 참고논문
[DeepMimic: example-guided deep reinforcement learning of physics-based character skills](https://github.com/CritBear/RL_AttackAnimation/blob/main/%5B%EA%B9%80%EA%B2%BD%EB%AF%BC%5D%20%EC%BA%A1%EC%8A%A4%ED%86%A4%EB%94%94%EC%9E%90%EC%9D%B8%20%EC%B5%9C%EC%A2%85%EB%B0%9C%ED%91%9C%EC%9E%90%EB%A3%8C.pdf)

### Problem
걷거나 물체를 집는 애니메이션에 비해 공격 애니메이션은 모든 관절을 움직여야 해서 다양한 모션을 만드는 비용이 크고, 자연스럽게 만들기 어렵기 때문에 주변 환경에 따라 유동적이고 자연스러운 애니메이션을 강화학습을 이용해 생성한다.

### Environment
환경은 Unity ML-Agents를 사용하였으며, 캐릭터의 관절은 Articulation Body(Unity 2020.3 이상부터 사용가능)를 사용했다. Agent의 학습은 Proximal Policy Optimization Algorithm으로 진행하였다.

### Train
Agent가 관측할 State는 캐릭터를 구성하는 각 Articulation Body의 Local Rotation, Linear Velocity, Angular Velocity로 구성되며, Action으로는 Articulation Body의 Y, Z, X Drive의 Target Angle, Force Limit을 Continuous actions으로 출력한다.

Reward는 크게 Task Reward, Imitation Reward로 이루어져 있는데, Task Reward는 타겟과의 거리가 짧을수록 크게, 타겟을 맞추면 애니메이션이 끝날 때까지 최대값을 부여하며, Imitation Reward는 각 관절이 참고하려는 모션과의 Local Rotation, Angular Velocity, End-effector position, Center-of-mass position 차이가 적을수록 큰 값을 부여한다.

### Results
1억 Steps를 학습한 결과
