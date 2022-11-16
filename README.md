Embedded_System_TermProject
==========
## 1. 제목 : Smart Blind
## 2. 목적 
#### 2.1 블라인드를 자동화함으로써 실생활에 편리함을 더 해주고 추가적인 기능을 제공하는 하드웨어를 개발
#### 2.2 Bluetooth와 같은 통신 기능을 기반으로 하는 IoT 디바이스를 개발한다.
#### 2.3 IoT 디바이스는 임베디드 실험 및 발표에서 배운 내용을 바탕으로 여러가지 센서와 보드를 활용한다.
  
## 3. 내용
#### 3.1) 블루투스를 통해 사용자의 취침시간과 기상시간을 입력받는다.
#### 3.2) 주변의 상황과 시간, 사용자의 명령에 따라 자동적으로 작동하는 블라인드를 개발한다. 블라인드의 작동 알고리즘은 아래와 같다.
##### - 취침 시간에는 블라인드를 내리고 조명을 끈다.
#####  - 기상 시간이 되었을 때, 외부가 밝으면 블라인드를 걷고, 어두우면 내린 채로 두며 조명을 켠다.
#####  - 블라인드가 닫혀 있을 때, 활동 시간(기상 시간 이후, 취침 시간 이전)동안 외부가 밝아지면 블라인드를 걷고 조명을 끈다.
#####  - 취침 시간이 되면 블라인드를 내린다.
#####  - 위의 내용을 반복한다.
#### 3.3) 버튼 5 개를 사용해 각각 블라인드 걷기, 블라인드 내리기, LED ON, LED OFF, auto 의 명령을 수행하도록 한다. 수동버튼을 사용자가 눌렀을 경우 위의 3-2 는 수행되지 않고, auto 버튼을 눌렀을 경우에 다시 수행된다.

## 4. 흐름도
![흐름도](https://user-images.githubusercontent.com/97718735/202163322-34e124fc-d41e-4561-ad67-c0b160329439.png)
