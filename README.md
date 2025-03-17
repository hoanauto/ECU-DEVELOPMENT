## AUTOMOTIVE ECU DEVELOPMENT 2025
<details><summary>LESSON 1: FIRST PROJECT WITH KEILC</summary>
    <p>
        
## LESSON 1: FIRST PROJECT WITH KEILC
### 1. Keilc
`KeilC` là một môi trường phát triển tích hợp (IDE) phổ biến được sử dụng để lập trình các vi điều khiển, đặc biệt là dòng vi điều khiển ARM Cortex-M, bao gồm STM32, LPC, Kinetis, và nhiều dòng vi điều khiển khác.
#### 1.1 Các thành phần chính của KeilC
- KeilC bao gồm các thành phần quan trọng sau:
  - µVision IDE: Giao diện tích hợp để viết, biên dịch và debug code.
  - C/C++ Compiler (Arm Compiler): Trình biên dịch mạnh mẽ giúp tối ưu hóa mã nguồn.
  - Debugger: Công cụ gỡ lỗi hỗ trợ mô phỏng và debug trực tiếp trên phần cứng.
  - RTX Real-Time Operating System (RTOS): Hệ điều hành thời gian thực cho vi điều khiển.
#### 1.2. Đặc điểm nổi bật của KeilC
- Hỗ trợ vi điều khiển ARM Cortex-M: KeilC là lựa chọn hàng đầu khi lập trình STM32.
- Không cần CubeMX: Người dùng có thể tự viết code mà không phụ thuộc vào các file tự động sinh của CubeMX.
- Hỗ trợ Debug mạnh mẽ: Cho phép debug qua JTAG, SWD, và giả lập.
- Thư viện phong phú: Cung cấp nhiều thư viện hỗ trợ phát triển ứng dụng nhúng.
- Giao diện trực quan: Dễ dàng cấu hình, quản lý dự án và biên dịch code.

![image](https://github.com/user-attachments/assets/343dceb4-8bb7-4e19-9910-e4676cbe6352)
### 2.Tạo project đầu tiên với Keilc
#### 2.1. Cài đặt
- Cài đặt Keil C về máy :https://www.keil.com/download/
- Cài đặt ST-link driver để nạp và debug: https://www.keil.com/download/
- Cài đặt thư viện chuẩn của STM32F103c8t6 cho keil: https://www.keil.arm.com/devices/
#### 2.2. Tạo project
- Ở màn hình chính của KeilC, trong mục Project, chọn New uVision Project, chọn thư mục lưu và đặt tên.
- Sau khi project được tạo, KeilC sẽ hiện cửa sổ để người dùng chọn dòng MCU sử dụng. Ở đây là STM32F103C8
![image](https://github.com/user-attachments/assets/e08a16bb-667c-477a-abb1-d470ee5c1cf7)
Nhấn OK, chuyển sang cửa sổ để thêm các files và thư viện cần thiết.
![image](https://github.com/user-attachments/assets/9f98fd97-8db1-4205-b58e-a181a07deb5e)
Tạo file main.c và add vào project.
![image](https://github.com/user-attachments/assets/18823c59-1d35-4f5d-bc40-b3fe784e76d0)
có thể thử build để kiểm tra xem có lỗi khi link thư viện không.
### 3. Blink Led PC13
Kit Bluepill có sẵn 1 user led trên board, led này nối tới chân 13 của GPIOC (chân PC13), nên chúng ta sẽ dùng nó để test hoạt động trong bài đầu tiên.
![image](https://github.com/user-attachments/assets/26fab549-ad5d-4b9e-ab35-c811514cf604)
- Để Blink led PC13 cần thực hiện 3 bước:
  - Cấp xung clock cho ngoại vi
  - Cấu hình chân của ngoại vi
  - Sử dụng ngoại v
#### 3.1. Cấp xung clock cho ngoại vi
Để cấu hình hoạt động cho các ngoại vi, trước hết cần cấu hình cấp xung clock cho chúng qua các bus.
- Các ngoại vi được cấp xung clock thông qua các đường bus sau:
  - `AHB (Advanced High Speed Buses)`: Đây là Bus kết nối hệ thống.
  - `APB1, APB2 (Advanced Peripheral Buses 1,2)`: Đây là các Bus kết nối với thiết bị ngoại vi và kết nối với hệ thống thông qua AHB.
-> Để cấu hình Clock cho một ngoại vi, chúng ta cần nắm được ngoại vi đó được cấp clock từ đường bus nào (AHB/APB). 
![image](https://github.com/user-attachments/assets/9cc317d1-ca02-4e6d-a194-83899628b0e5)
- Nhìn sơ đồ ta thấy được, **các GPIO được cấp clock từ APB2**
- **RCC (Reset and Clock Control)** là bộ điều khiển xung nhịp trên STM32, chịu trách nhiệm quản lý nguồn xung nhịp cho CPU, GPIO, bộ nhớ và các ngoại vi khác. Trong STM32F103, RCC có địa chỉ cơ bản là `0x40021000`.
- **APB2** được cấu hình bởi **thanh ghi APB2 peripheral clock enable register (RCC_APB2ENR)**. 
![image](https://github.com/user-attachments/assets/d5bb76c8-7c0d-4bf0-aa03-5f37fa297e8e)
- **thanh ghi APB2 peripheral clock enable register (RCC_APB2ENR)** được định nghĩa với `offset` (độ lệch) so với địa chỉ cơ bản là `0x18`
--> địa chỉ đầy đủ của thanh ghi RCC_APB2ENR là `0x40021018` và được định nghĩa như sau
  ```c
  #define RCC_APB2ENR *((unsigned int*)0x40021018)
  
![image](https://github.com/user-attachments/assets/10539415-44f1-446d-b233-c179ee737c80)
- Set bit IOPCEN lên 1 để cấp clock cho GPIOC.
  ```c
    RCC_APB2ENR |= (1<<4);
#### 3.2. Cấu hình chế độ hoạt động
- Sau khi cấp clock cho GPIOC, cần cấu hình chân (xác định chân và chế độ hoạt động của chân đó, cụ thể là PC13) trong thanh ghi Port configuration register
**Port configuration register low (GPIOx_CRL)**: cấu hình cho các chân từ 0-7 trong Portx
![image](https://github.com/user-attachments/assets/28b528aa-24d6-4881-9294-621d25421837)
**Port configuration register high (GPIOx_CRH)**: cấu hình cho các chân từ 8-15 trong Portx
![image](https://github.com/user-attachments/assets/260c7bb5-897c-4e58-9a2a-00935a498009)
- địa chỉ đầy đủ của thanh ghi GPIOC_CRH là `0x40011004` và được định nghĩa như sau:
  ```c
      #define GPIOC_CRH *((unsigned int *) 0x40011004)// dia chi portc
  
- Các cặp bit CNFy cùng với các cặp bit MODEy tương ứng giúp xác định chế độ hoạt động và các thông số của từng chân. 
- Trong bài này, ta cần cấu hình cho PC13 làm ngõ ra chân Output, Push - pull, tốc độ 50mHz --> nên ta cấu hình thanh ghi GPIOC_CRH, **MODE13** có cặp bit **11** và **CNF13** với cặp bit **00**
![image](https://github.com/user-attachments/assets/1d14befe-d261-4f2f-87f7-b86b774e4697)
    ```c
    GPIOC_CRH &= ~((1<<23)|(1<<22)); // ~(1:1) = (0:0)
	GPIOC_CRH |= ((1<<21)|(1<<20));
#### 3.3. Ghi giá trị 
- Sau khi đã cấu hình xong, tiến hành ghi giá trị ra chân PC13 để điều khiển Led. Trạng thái các chân trên Port tương ứng được xác định bởi các bit trong thanh ghi Port output data register (GPIOx_ODR). 
- Bằng cách thay đổi giá trị Bit ODR13 trong thanh ghi này, chúng ta có thể điều khiển trạng thái Led ở chân PC13.
- Ví dụ, có thể điều khiển led nhấp nháy sau 1 khoảng thời gian bằng các lệnh sau.
    ```c
    while(1){

    	GPIOC->ODR |= 1<<13; // off
    	delay(10000000);
    	GPIOC->ODR &= ~(1<<13); // on
    	delay(10000000);
        }
- Hàm delay() được viết như sau:
    ```c
    void delay(__IO uint32_t timedelay){ 
	for(int i=0; i<timedelay; i++){}
    }
### 4. Xây dựng cấu trúc thanh ghi của các ngoại vi
- Thay vì dùng #define, ta có thể dùng struct để nhóm tất cả các thanh ghi RCC và GPIO vào một cấu trúc:
    ```c
    typedef struct
    {
    unsigned int CRL;
    unsigned int CRH;
    unsigned int IDR;
    unsigned int ODR;
    unsigned int BSRR;
    unsigned int BRR;
    unsigned int LCKR;
    } GPIO_TypeDef;

    
    typedef struct
    {
     volatile unsigned int CR;         // Địa chỉ offset: 0x00
    volatile unsigned int CFGR;       // Địa chỉ offset: 0x04
    volatile unsigned int CIR;        // Địa chỉ offset: 0x08
    volatile unsigned int APB2RSTR;   // Địa chỉ offset: 0x0C
    volatile unsigned int APB1RSTR;   // Địa chỉ offset: 0x10
    volatile unsigned int AHBENR;     // Địa chỉ offset: 0x14
    volatile unsigned int APB2ENR;    // Địa chỉ offset: 0x18
    volatile unsigned int APB1ENR;    // Địa chỉ offset: 0x1C
    volatile unsigned int BDCR;       // Địa chỉ offset: 0x20
    volatile unsigned int CSR;        // Địa chỉ offset: 0x24
    } RCC_TypeDef; // 1 phan tu chiem 32 bit, phan tu tiep theo cung chiem 32bit tiep theo
- Với cách này, ta có thể truy cập thanh ghi như truy cập thuộc tính của struct:
  	```c
  	 #define RCC ((RCC_TypeDef *) 0x40021000) // dia chi cua struct
	#define GPIOC ((GPIO_TypeDef *) 0x40011000) // dia chi port c
	RCC->APB2ENR |= (1 << 4);  // Bật Clock cho GPIOC
### 5. Tổng kết và mở rộng
- Việc code trên thanh ghi nhằm giúp các bạn hiểu rõ cách hoạt động chi tiết của từng ngoại vi, cũng như tăng hiệu suất của chương trình.
- Tuy nhiên, việc lập trình thanh ghi có thể trở nên khá phức tạp
	```c
 	void WritePin(GPIO_TypeDef *GPIO_Port, uint8_t Pin, uint8_t state)
	{
	if(state == HIGH)
	GPIO_Port->ODR |= (1 << Pin);
	else
	GPIO_Port->ODR &= ~(1 << Pin);
	}
	
	void GPIO_Config(void){			
 	   GPIOC->CRH |= GPIO_CRH_MODE13_0; 	//MODE13[1:0] = 11: Output mode, max speed 50 MHz
 	   GPIOC->CRH |= GPIO_CRH_MODE13_1; 	
 	   GPIOC->CRH &= ~GPIO_CRH_CNF13_0;	              //CNF13[1:0] = 00: General purpose output push-pull
   	 GPIOC->CRH &= ~GPIO_CRH_CNF13_1;
	}
 ### 6. Đọc trạng thái nút nhấn
 - Sơ đồ đọc trạng thái
![image](https://github.com/user-attachments/assets/429cbb1b-0092-46f7-b1fe-a155366b30a6)
- Cấp clock cho ngoại vi: cấp cho GPIOA và GPIOC
  	```c
	RCC -> APB2ENR |= ((1<<4)|(1<<2));// bit 1 o vi tri thu 4 va 2, con lai bang 0 (GPIOC va GPIOA)
- Cấu hình chế độ chân:
  ![image](https://github.com/user-attachments/assets/507f207f-630e-4922-ae86-34b16ece68d7)
	```c
 	GPIOC -> CRH&= ~((1<<23)|(1<<22)); // ~(1:1) = (0:0)
	GPIOC ->CRH|= ((1<<21)|(1<<20));
	// PA0
	GPIOA-> CRL &= ~((1<<0)|(1<<1)|(1<<2));
	GPIOA ->CRL |=  (1<<3);
	GPIOA -> ODR |= 1; // pull up or pull down
 - sử dụng ngoại vi:
	```c
	   while(1){
		if((GPIOA->IDR & (1 << 0)) == 0) // Đọc trạng thái nút nhấn
		{
			GPIOC->ODR = 0 << 13;   // Nếu PA0 = 0 -> PC13 = 0
		}
		else
		{
			GPIOC->ODR = 1 << 13;   // Nếu PA0 = 1 -> PC13 = 1
		}
	}
</details>

<details><summary>LESSON 2: GPIO</summary>
    <p>	
	    
 ## LESSON 2: GPIO

