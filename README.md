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
 ### 1. Thư viện STM32F10x Standard Peripherals Firmware Library 
- Là 1 thư viện hoàn chỉnh được phát triển cho dòng Stm32. Bao gồm đầy đủ driver cho tất cả các ngoại vi tiêu chuẩn.
- Thư viện này bao gồm các hàm, cấu trúc dữ liệu và macro của các tính năng thiết bị ngoại vi STM32. 
- Cấu trúc thư viện:
	- stm32f10x.h – Header chính của thư viện.
	- Thư viện con (cho từng ngoại vi):
		- stm32f10x_rcc.h – Quản lý clock
		- stm32f10x_gpio.h – Quản lý GPIO
		- stm32f10x_usart.h – Quản lý UART
		- stm32f10x_tim.h – Quản lý Timer
		- stm32f10x_adc.h – Quản lý ADC
		- stm32f10x_exti.h – Quản lý ngắt ngoài
		- … và nhiều thư viện khác
### 2. Cấu hình và sử dụng ngoại vi (GPIO)
- Thư viện SPL cung cấp các hàm và các định nghĩa giúp việc cấu hình và sử dụng ngoại vi dễ dàng và rõ ràng.
- Các hàm phục vụ cho việc cấu hình GPIO, cấp xung ngoại vi được định nghĩa trong file `"stm32f10x_rcc.h"`, và `"stm32f10x_gpio.h"`. Ở trong thư viện này, các cấu hình được chia thành các trường và định nghĩa bằng các cấu trúc như struct và enum.
- chúng ta vẫn sẽ theo 3 bước: Cấp clock ngoại vi, cấu hình và sử dụng.
![image](https://github.com/user-attachments/assets/0470103f-455f-456b-a137-60faaecc33b7)
#### 2.1. Cấp xung clock cho GPIO
- Các hàm có chức năng cấp xung hoặc ngừng cấp xung cho ngoại vi tương ứng. Các hàm này được định nghĩa trong file "stm32f10x_rcc.h". 
- Các hàm này nhận tham số vào là Macro của các ngoại vi được định nghĩa sẵn trong file header, tham số thứ 2 quy định việc cấp hay ngưng xung clock cho ngoại vi tương ứng.
- **RCC_APB1PeriphClockCmd** //Enables or disables the Low Speed APB (APB1) peripheral clock. 
- **RCC_APB2PeriphClockCmd** // Enables or disables the High Speed APB (APB2) peripheral clock.
- **RCC_AHBPeriphClockCmd**
- Trong bài này sử dụng led PC13, nên cấp xung cho GPIOC qua Bus APB2
 	 ```c
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOC, ENABLE); // để cấu hình clock.
- Viết vào hàm RCC_Config() để gọi hàm cấp xung clock 
	```c
	void RCC_Config(){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
#### 2.2. Cấu hình GPIO
- Trong thư viện SPL, các thuộc tính của GPIO được tổ chức thành 1 struct GPIO_InitTypeDef chứa các trường GPIO_Mode, GPIO_Pin và GPIO_Speed.
  	```c
	typedef struct
	{
	  uint16_t GPIO_Pin;             /*!< Specifies the GPIO pins to be configured.
	                                      This parameter can be any value of @ref GPIO_pins_define */
	
	  GPIOSpeed_TypeDef GPIO_Speed;  /*!< Specifies the speed for the selected pins.
	                                      This parameter can be a value of @ref GPIOSpeed_TypeDef */
	
	  GPIOMode_TypeDef GPIO_Mode;    /*!< Specifies the operating mode for the selected pins.
	                                      This parameter can be a value of @ref GPIOMode_TypeDef */
	}GPIO_InitTypeDef;

- các thuộc tính của 1 chân trong GPIO có thể được cấu hình thông qua struct GPIO_InitTypeDef, chúng ta sẽ tạo 1 biến struct kiểu này, sau đó gán các giá trị cần cấu hình thông qua biến đó.
 	```c
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APBxPeriph_GPIOx, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_xx;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_xx;
	GPIO_Init(GPIOx, &GPIO_InitStruct);
 - khởi tạo GPIOx với các tham số đã được thiết lập trong GPIO_InitStruct. Hàm nhận 2 tham số là 1 GPIOx cần khởi tạo và 1 con trỏ trỏ tới struct GPIO_InitTypedDef chứa các thông tin đã thiết lập cho GPIO. 
- Vì vậy, để khởi tạo 1 GPIO để sử dụng, trước tiên cần cấu hình clock, sau đó tạo 1 struct GPIO_InitTypedDef  cấu hình tham số cho GPIO, sau đó gọi hàm GPIO_Init() với GPIOx cần cấu hình và struct vừa tạo.

##### 2.2.1. GPIO_Pin
- GPIO_Pin là trường xác định chân trong GPIOx tương ứng. các giá trị được khai báo trong file header, có dạng GPIO_Pin_x với x là chân từ 0-15.
![image](https://github.com/user-attachments/assets/86ad598b-be52-4735-b455-e2957d3acc1b)
##### 2.2.2. GPIO_Speed
- GPIO_Speed là trường xác định tốc độ đáp ứng của chân. Thường được cấu hình đi kèm với chế độ Output, các giá trị cũng được khai báo trong file header trong GPIO_SpeedTypeDef:
	```c
 	typedef enum
	{ 
	  GPIO_Speed_10MHz = 1,
	  GPIO_Speed_2MHz, 
	  GPIO_Speed_50MHz
	}GPIOSpeed_TypeDef;
##### 2.2.3. GPIO Mode
- GPIO_Mode là một trường dùng để xác định chế độ hoạt động của chân GPIO trong thư viện của STM32. 
	```c
	 typedef enum
	{ GPIO_Mode_AIN = 0x0,
	  GPIO_Mode_IN_FLOATING = 0x04,
	  GPIO_Mode_IPD = 0x28,
	  GPIO_Mode_IPU = 0x48,
	  GPIO_Mode_Out_OD = 0x14,
	  GPIO_Mode_Out_PP = 0x10,
	  GPIO_Mode_AF_OD = 0x1C,
	  GPIO_Mode_AF_PP = 0x18
	}GPIOMode_TypeDef;
- GPIO_Mode_AIN:
	- Mô tả: Analog Input.
	- Giải thích: Chân GPIO được cấu hình làm đầu vào analog. Thường được sử dụng cho các chức năng như ADC (Analog to Digital Converter).
- GPIO_Mode_IN_FLOATING:
	- Mô tả: Floating Input.
	- Giải thích: Chân GPIO được cấu hình làm đầu vào và ở trạng thái nổi (không pull-up hay pull-down). Điều này có nghĩa là chân không được kết nối cố định với mức cao (VDD) hoặc mức thấp (GND) thông qua điện trở.
- GPIO_Mode_IPD:
 	- Mô tả: Input with Pull-down.
	- Giải thích: Chân GPIO được cấu hình làm đầu vào với một điện trở pull-down nội bộ kích hoạt. Khi không có tín hiệu nào được áp dụng lên chân này, nó sẽ được kéo về mức thấp (GND).
- GPIO_Mode_IPU:
	- Mô tả: Input with Pull-up.
	- Giải thích: Chân GPIO được cấu hình làm đầu vào với một điện trở pull-up nội bộ kích hoạt. Khi không có tín hiệu nào được áp dụng lên chân này, nó sẽ được kéo về mức cao (VDD).
 - GPIO_Mode_Out_OD:
	- Mô tả: Open-drain Output.
	- Giải thích: Chân GPIO được cấu hình làm đầu ra với chế độ open-drain. Trong chế độ này, chân có thể được kéo xuống mức thấp, nhưng để đạt được mức cao, cần một điện trở pull-up ngoài hoặc từ một nguồn khác.
 - GPIO_Mode_Out_PP:
	- Mô tả: Push-pull Output.
	- Giải thích: Chân GPIO được cấu hình làm đầu ra với chế độ push-pull. Trong chế độ này, chân có thể đạt được cả mức cao và mức thấp mà không cần bất kỳ phần cứng bổ sung nào.
- GPIO_Mode_AF_OD:
	- Mô tả: Alternate Function Open-drain.
	- Giải thích: Chân GPIO được cấu hình để hoạt động trong một chức năng thay thế (như USART, I2C, etc.) và sử dụng chế độ open-drain.
- GPIO_Mode_AF_PP:
	 - Mô tả: Alternate Function Push-pull.
	- Giải thích: Chân GPIO được cấu hình để hoạt động trong một chức năng thay thế và sử dụng chế độ push-pull.
#### 2.3. Các hàm cơ bản trên GPIO
- Thư viện SPL hỗ trợ sẵn các hàm để thực thi trên các GPIO.
- **GPIO_SetBits(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin)**
		-  Mô tả: Đặt một hoặc nhiều chân GPIO ở mức cao (logic 1).
		- Tham số:
			- GPIOx: là cổng GPIO muốn điều khiển (ví dụ: GPIOA, GPIOB,...).
			- GPIO_Pin: chọn chân hoặc chân cần đặt ở mức cao (ví dụ: GPIO_Pin_0, GPIO_Pin_1 hoặc kết hợp như GPIO_Pin_0 | GPIO_Pin_1).
 - **GPIO_ResetBits(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin)**
	- Mô tả: Đặt một hoặc nhiều chân GPIO ở mức thấp (logic 0).
	- Tham số: Tương tự như hàm GPIO_SetBits.
- **GPIO_ReadInputDataBit(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin)**
	- Mô tả: Đọc trạng thái của một chân GPIO đã được cấu hình là input.
	- Tham số: Tương tự như hàm GPIO_SetBits.
	- Giá trị trả về: Trả về Bit_SET nếu chân đang ở mức cao hoặc Bit_RESET nếu chân đang ở mức thấp.
- **GPIO_ReadOutputDataBit(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin)**
	- Mô tả: Đọc trạng thái của một chân GPIO đã được cấu hình là output.
	- Tham số: Tương tự như hàm GPIO_SetBits.
	- Giá trị trả về: Trả về Bit_SET nếu chân đang ở mức cao hoặc Bit_RESET nếu chân đang ở mức thấp.
- **GPIO_WriteBit(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin, BitAction BitVal)**
	- Mô tả: Đặt trạng thái của một chân GPIO dựa trên giá trị của BitVal.
	- Tham số:
		- GPIOx và GPIO_Pin tương tự như hàm GPIO_SetBits.
		- BitVal: là giá trị mà bạn muốn đặt cho chân GPIO, có thể là Bit_SET hoặc Bit_RESET.
- **GPIO_ReadInputData(GPIO_TypeDef GPIOx)**
	- Mô tả: Đọc giá trị của tất cả các chân GPIO đã được cấu hình là đầu vào trên cổng GPIO chỉ định.
	- Tham số:
		- GPIOx: cổng GPIO mà bạn muốn đọc (ví dụ: GPIOA, GPIOB,...).
		- Giá trị trả về: Một giá trị 16-bit biểu diễn trạng thái của tất cả các chân trên cổng GPIO.
 - **GPIO_ReadOutputData(GPIO_TypeDef GPIOx)**
	- Mô tả: Đọc giá trị của tất cả các chân GPIO đã được cấu hình là đầu ra trên cổng GPIO chỉ định.
	- Tham số:
		- GPIOx: cổng GPIO mà bạn muốn đọc.
		- Giá trị trả về: Một giá trị 16-bit biểu diễn trạng thái của tất cả các chân trên cổng GPIO.
 - **GPIO_Write(GPIO_TypeDef GPIOx, uint16_t PortVal)**
	- Mô tả: Ghi giá trị cho toàn bộ cổng GPIO.
	- Tham số:
		- GPIOx: cổng GPIO bạn muốn ghi.
		- PortVal: giá trị 16-bit mà bạn muốn đặt cho cổng GPIO.
- **GPIO_PinLockConfig(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin)**
	- Mô tả: Khóa cấu hình của chân GPIO. Sau khi chân đã bị khóa, bạn sẽ không thể thay đổi cấu hình của nó cho đến khi hệ thống được reset.
	- Tham số:
		- GPIOx: cổng GPIO mà bạn muốn khóa chân.
		- GPIO_Pin: chọn chân cần khóa (ví dụ: GPIO_Pin_0, GPIO_Pin_1 hoặc kết hợp như GPIO_Pin_0 | GPIO_Pin_1).
- **GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)**
	- Mô tả: Cấu hình chân sự kiện đầu ra.
	- Tham số:
		- GPIO_PortSource: xác định cổng GPIO.
		- GPIO_PinSource: xác định chân GPIO.
- **GPIO_EventOutputCmd(FunctionalState NewState)**
	- Mô tả: Cho phép hoặc vô hiệu hóa chân sự kiện đầu ra.
	- Tham số:
	- NewState: trạng thái mới của chân. Có thể là ENABLE hoặc DISABLE.
 - **GPIO_AFIODeInit()**
	- Mô tả: Đặt lại tất cả các thanh ghi của AFIO (Alternate Function IO) về giá trị mặc định.
</details>

<details><summary>LESSON 3: INTERRUPT - TIMER</summary>
    <p>	
	    
 ## LESSON 3: INTERRUPT - TIMER
 ### 1. Định nghĩa ngắt (Interrupt)
 - Ngắt (Interrupt) là một cơ chế trong vi điều khiển và vi xử lý giúp tạm dừng chương trình chính để xử lý một sự kiện quan trọng, sau đó quay lại tiếp tục chương trình chính như bình thường.
- Ví dụ:
	- Khi bạn nhấn một nút trên bàn phím, vi điều khiển có thể nhận một ngắt để đọc dữ liệu phím.
	- Khi có dữ liệu đến từ cổng UART, vi điều khiển nhận ngắt để xử lý dữ liệu ngay lập tức.
![image](https://github.com/user-attachments/assets/5277f523-fc4e-46a6-9e47-aaa1052d9478)

- Mỗi ngắt có 1 trình phục vụ ngắt, sẽ yêu cầu MCU thực thi lệnh tại trình phục vụ ngắt khi có ngắt xảy ra.
- Các ngắt có các địa chỉ cố định trong bộ nhớ để giữ các trình phục vụ. Các địa chỉ này gọi là vector ngắt.
- Các loại ngắt phổ biến trong STM32F1:
	- **Ngắt bên ngoài (EXTI - External Interrupt)**: Kích hoạt khi có tín hiệu từ chân GPIO.
	- **Ngắt từ Timer**: Kích hoạt khi bộ định thời (Timer) đến một giá trị nhất định.
	- **Ngắt từ giao tiếp ngoại vi (USART, I2C, SPI, CAN, v.v.)**: Xảy ra khi có dữ liệu đến hoặc cần xử lý.
	- Ngắt do lỗi (HardFault, BusFault, UsageFault, v.v.): Kích hoạt khi có lỗi trong chương trình.
![image](https://github.com/user-attachments/assets/959aac63-9727-4f7c-b82b-35cdf6f67928)
### 2. Các loại ngắt thông dụng
- Thanh ghi PC (Program Counter) là một thanh ghi đặc biệt trong vi điều khiển/vi xử lý, dùng để lưu địa chỉ lệnh tiếp theo sẽ được thực thi trong chương trình.
- Chức năng của thanh ghi PC:
	- Khi vi điều khiển thực hiện một lệnh, thanh ghi PC sẽ tự động tăng lên để trỏ đến lệnh kế tiếp.
	- Khi xảy ra nhảy lệnh (branch, jump, call, return, interrupt,...), giá trị của PC sẽ được thay đổi để trỏ đến địa chỉ lệnh cần thực thi tiếp theo.
	- Khi có ngắt (interrupt), giá trị của PC sẽ được lưu vào stack trước khi nhảy vào hàm xử lý ngắt. Sau khi xử lý xong, PC được khôi phục để tiếp tục chương trình chính.
   
![image](https://github.com/user-attachments/assets/8cd68f5f-892f-46d4-b116-588dff6f5750)
#### 2.1. Ngắt ngoài
- Ngắt ngoài (EXTI - External Interrupt) là cơ chế giúp vi điều khiển phản hồi ngay lập tức khi có tín hiệu từ bên ngoài thay đổi (ví dụ: nhấn nút, tín hiệu từ cảm biến).
  
 ![image](https://github.com/user-attachments/assets/da27885f-1dcf-482b-90cd-5c31d74243f9)

 - Xảy ra khi có thay đổi điện áp trên các chân GPIO được cấu hình làm ngõ vào ngắt.
	- LOW: kích hoạt ngắt liên tục khi chân ở mức thấp.
	- HIGH: Kích hoạt liên tục khi chân ở mức cao.
	- Rising: Kích hoạt khi trạng thái trên chân chuyển từ thấp lên cao.
	- Falling: Kích hoạt khi trạng thái trên chân chuyển từ cao xuống thấp.
 ![image](https://github.com/user-attachments/assets/178d0565-eb93-4d2d-82c4-028c14e57eb8)
#### 2.2. Ngắt Timer
- Ngắt Timer là một trong những tính năng quan trọng giúp vi điều khiển thực thi một tác vụ theo chu kỳ thời gian mà không cần sử dụng vòng lặp liên tục (polling).
- Nguyên lý hoạt động của ngắt Timer
	- Timer trong STM32F1 có thể tự động tăng giá trị **CNT (Counter)** theo xung clock.
Khi giá trị CNT đạt đến giá trị **ARR (Auto-Reload Register)** đã cấu hình, nó sẽ kích hoạt ngắt Timer.
	- Sau đó, CPU sẽ nhảy vào **hàm xử lý ngắt (ISR - Interrupt Service Routine)** để thực thi nhiệm vụ.
- Ví dụ:
	- Cấu hình Timer 2 với chu kỳ 1ms, mỗi lần Timer đạt 1ms → kích hoạt ngắt.
	- Trong ngắt, ta có thể đếm số lần gọi để thực hiện tác vụ mỗi giây.
 - Xảy ra khi giá trị trong thanh ghi đếm của timer bị tràn. Sau mỗi lần tràn, cần phải reset giá trị thanh ghi để có thể tạo ngắt tiếp theo.
- Cấu trúc Timer trong STM32F1: STM32F1 có nhiều bộ Timer, trong đó phổ biến nhất là:
	- Timer 1: Timer nâng cao (Advanced Timer)
	- Timer 2, 3, 4: Timer chung (General Purpose Timer)
	- Timer 6, 7: Timer cơ bản (Basic Timer)
- Mỗi Timer có một thanh ghi đếm (CNT) và một thanh ghi ARR để xác định khi nào ngắt xảy ra.
![image](https://github.com/user-attachments/assets/f80b4d63-f0ff-4911-9fdf-ade967d2d38d)
#### 2.3. Ngắt truyền thông
- Xảy ra khi có sự kiện truyền/nhận dữ liệu giữa MCU và các thiết bị khác, thường sử dụng cho các giao thức như UART, SPI, I2C để đảm bảo việc truyền/nhận được chính xác.
![image](https://github.com/user-attachments/assets/101089dc-02d7-486e-a1b8-c66491c55a7e)
### 3. Mức độ ưu tiên ngắt
- Độ ưu tiên ngắt là khác nhau ở các ngắt. Nó xác định ngắt nào được quyền thực thi khi nhiều ngắt xảy ra đồng thời.
- STM32 quy định ngắt nào có số thứ tự ưu tiên càng thấp thì có quyền càng cao. Các ưu tiên ngắt có thể lập trình được.
### 4. Timer
#### 4.1. Định nghĩa Timer
- Có thể hiểu 1 cách đơn giản: timer là 1 mạch digital logic có vai trò đếm mỗi chu kỳ clock (đếm lên hoặc đếm xuống).
- Timer còn có thể hoạt động ở chế độ nhận xung clock từ các tín hiệu ngoài. Có thể là từ 1 nút nhấn, bộ đếm sẽ được tăng sau mỗi lần bấm nút (sườn lên hoặc sườn xuống tùy vào cấu hình). Ngoài ra còn các chế độ khác như PWM, định thời …vv.
- STM32f103C8 có tất cả 7 timer nhưng trong đó đã bao gồm 1 systick timer, 2 watchdog timer. Vậy chỉ còn lại 4 timerz dùng cho các chức năng như ngắt, timer base, PWM, Encoder, Input capture…. Trong đó TIM1 là Timer đặc biệt, chuyên dụng cho việc xuất xung với các mode xuất xung, các mode bảo vệ đầy đủ hơn so với các timer khác. TIM1 thuộc khối clock APB2, còn các TIM2,TIM3,TIM4 thuộc nhóm APB1.
![image](https://github.com/user-attachments/assets/4c3a7491-7819-43b2-b01a-d6314533245b)
- Timer hoạt động bằng cách đếm xung clock.
- Giá trị hiện tại của bộ đếm được lưu trong thanh ghi CNT (Counter Register).
- Khi CNT đạt đến giá trị ARR (Auto-Reload Register), có thể kích hoạt ngắt hoặc thay đổi trạng thái của chân đầu ra.Giá trị bộ đếm này được cài đặt tối đa là 16bit tương ứng với giá trị là 65535.
- Bộ chia tần số (Prescaler - PSC) giúp điều chỉnh tốc độ đếm của Timer. Bộ chia này có giá trị tối đa là 16 bit tương ứng với giá trị là 65535.
- ![image](https://github.com/user-attachments/assets/6adb3394-f86a-44bd-a20a-d2b7dab84d25)
- Tần số sau bộ chia này sẽ được tính là: fCK_CNT = fCK_PSC/(PSC+1).
	- FCK_CNT: tần số sau bộ chia.
	- fCK_PSC: tần số clock đầu vào cấp cho timer.
	- PSC: chính là giá trị truyền vào được lập trình bằng phần mềm
 	```c
	FTIMER= fSYSTEM/[(PSC+1)(Period+1)]
- Ftimer : là giá trị cuối cùng của bài toán, đơn vị là hz.
- F system : tần số clock hệ thống được chia cho timer sử dụng, đơn vị là hz.
- PSC : giá trị nạp vào cho bộ chia tần số của timer. Tối đa là 65535.
- Period : giá trị bộ đếm nạp vào cho timer. Tối đa là 65535.
#### 4.2. Cấu hình Timer
- Tương tự các ngoại vi khác, cần xác định clock cấp cho timer, các tham số cho timer được cấu hình trong struct TIM_TimBaseInitTypeDef, cuối cùng gọi hàm TIM_TimBaseInit() để khởi tạo timer.
	```c
 	typedef struct
	{
	  uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
	                                       This parameter can be a number between 0x0000 and 0xFFFF */
	
	  uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
	                                       This parameter can be a value of @ref TIM_Counter_Mode */
	
	  uint16_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
	                                       Auto-Reload Register at the next update event.
	                                       This parameter must be a number between 0x0000 and 0xFFFF.  */ 
	
	  uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
	                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */
	
	} TIM_TimeBaseInitTypeDef;
- TIM_TimeBaseInitTypeDef gồm:

	- Clock division
	- Prescaler
	- Period 
	- Mode
 - ví dụ:  
	- fSystem 72Mhz, 1s tạo 72 triệu dao động.
	- TimerClock = fSystem/Clock_Division
	- Giá trị prescaler quy định số dao động mà sau đó timer sẽ đếm lên 1 lần.
	- 1 dao động mất 1/72000000 (s) 
	```c
 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitStruct.TIM_Prescaler = 7200-1;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM2, ENABLE);
</details>

<details><summary>LESSON 4: COMMUNICATION PROTOCOLS</summary>
    <p>
	    
## LESSON 4: COMMUNICATION PROTOCOLS
- Các MCU truyền nhận dữ liệu với nhau hoặc với các thiết bị thông qua tín hiệu điện áp. MCU có thể truyền nhận song song, nối tiếp các tín hiệu điện áp này thông quá các chân được cấu hình riêng biệt.
![image](https://github.com/user-attachments/assets/00005aaf-7efc-44cd-80f1-a79179eb6a42)
- Giao tiếp song song: Truyền nhiều bit cùng lúc, tốc độ cao nhưng tốn nhiều chân GPIO (VD: giao tiếp với RAM, LCD).
- Giao tiếp nối tiếp: Truyền từng bit một theo thời gian (các bit nối đuôi nhau), tốc độ chậm hơn nhưng tiết kiệm chân GPIO (VD: UART, I2C, SPI).
- Vấn đề về các bit giống nhau liền kề, MCU không phân biệt được 2 bit giống nhau liên tiếp => phải dùng các chuẩn giao tiếp
### 1. SPI
#### 1.1. Đặc điểm
- SPI (Serial Peripheral Interface) là một giao thức giao tiếp nối tiếp đồng bộ tốc độ cao giữa `Master và Slave`.
- Có các đặt điểm như sau:
	- Chuẩn giao tiếp nối tiếp
	- Đồng bộ: cần clock (SCK) chung.
	- Hoạt động ở chế độ song công: Nghĩa là tại 1 thời điểm có thể xảy ra đồng thời quá trình truyền và nhận. Là giao tiếp đồng bộ, bất cứ quá trình nào cũng đều được đồng bộ với xung clock sinh ra bởi thiết bị Master. 
	- Một Master có thể giao tiếp với nhiều Slave
   	- Tốc độ truyền thông cao: SPI cho phép truyền dữ liệu với tốc độ rất nhanh, thường đạt được tốc độ Mbps hoặc thậm chí hàng chục Mbps. Điều này rất hữu ích khi cần truyền dữ liệu nhanh và đáng tin cậy trong các ứng dụng như truyền thông không dây, điều khiển từ xa và truyền dữ liệu đa phương tiện.
- Sử dụng 4 dây giao tiếp:
  ![image](https://github.com/user-attachments/assets/c0bb60dc-38a2-476c-bc46-25e64479cf4e)

- **SCK (Serial Clock)**: Thiết bị Master tạo xung tín hiệu SCK và cung cấp cho Slave. Xung này có chức năng giữ nhịp cho giao tiếp SPI. Mỗi nhịp trên chân SCK báo 1 bit dữ liệu đến hoặc đi 
-**MISO (Master Input Slave Output)**: Tín hiệu tạo bởi thiết bị Slave và nhận bởi thiết bị Master.
- **MOSI (Master Output Slave Input)**: Tín hiệu tạo bởi thiết bị Master và nhận bởi thiết bị Slave. 
- **SS (Đôi khi CS- Slave Select/Chip Select)**: Chọn thiết bị Slave cụ thể để giao tiếp. Để chọn Slave giao tiếp thiết bị Master chủ động kéo đường SS tương ứng xuống mức 0 (Low). 
- SPI cho phép 1 MCU chủ giao tiếp với nhiều thiết bị tớ thông qua tín hiệu chọn thiết bị SS. Các thiết bị tớ chỉ có thể có 1 chân CS để nhận tín hiệu chọn này, tuy nhiên thiết bị chủ có thể có nhiều hơn 1 chân SS để chọn từng thiết bị muốn giao tiếp.
![image](https://github.com/user-attachments/assets/f79785fa-df83-4050-9231-d2495b5f705a)
#### 1.2. Khung truyền SPI
- Mỗi chip Master hay Slave đều có một thanh ghi dữ liệu 8 bits. Quá trình truyền nhận giữa Master và Slave xảy ra đồng thời theo chu kỳ clock ở chân CLK, một byte dữ liệu được truyền theo cả 2 hướng 
- Bắt đầu quá trình, master sẽ kéo chân CS của slave muốn giao tiếp xuống 0 để báo hiệu muốn truyền nhận.
- Clock sẽ được cấp bởi master, tùy vào chế độ được cài, với mỗi xung clock,  1 bit sẽ được truyền từ master đến slave và slave cũng truyền 1 bit cho master.
- Các thanh ghi cập nhật giá trị và dịch 1 bit. Như vậy sau 8 chu kỳ clock thì hoàn tất việc truyền và nhận 1 byte dữ liệu.
- Lặp lại quá trình trên đến khi truyền xong 8 bit trong thanh ghi.

![image](https://github.com/user-attachments/assets/cf5062d9-617c-425e-9d84-f2f5e3d07efd)
#### 1.3. Các chế độ
- SPI có 4 chế độ hoạt động phụ thuộc vào cực của xung giữ (Clock Polarity – CPOL) và pha (Phase - CPHA). 
- CPOL dùng để chỉ trạng thái của chân SCK ở trạng thái nghỉ. Chân SCK giữ ở mức cao khi CPOL=1 (không truyền dữ liệu khi SCK =1) hoặc mức thấp khi CPOL=0 (không truyền dữ liệu khi SCK = 0).
- CPHA dùng để chỉ các mà dữ liệu được lấy mẫu theo xung. CPHA = 0 (đọc dữ liệu ở cạnh 1, truyền dữ liệu ở cạnh 2), CPHA = 1(đọc dữ liệu ở cạnh 2, truyền dữ liệu ở cạnh 1)
  ![image](https://github.com/user-attachments/assets/d08899e7-412f-4e28-ab5f-a706bc82407f)
### 2. I2C
#### 2.1. Đặc điểm
- I2C (Inter-Integrated Circuit) là một giao thức giao tiếp nối tiếp đồng bộ được phát triển bởi Philips, giúp vi điều khiển giao tiếp với nhiều thiết bị chỉ bằng 2 dây.
- Ưu điểm:
	- Tiết kiệm chân GPIO (chỉ cần 2 dây để giao tiếp nhiều thiết bị).
	- Hỗ trợ nhiều thiết bị trên cùng một bus (đa Master - đa Slave).
	- Có thể kéo dài khoảng cách giữa các thiết bị hơn so với SPI
- Chuẩn giao tiếp nối tiếp
- Đồng bộ: do đó đầu ra của các bit được đồng bộ hóa với việc lấy mẫu các bit bởi một tín hiệu xung nhịp được chia sẻ giữa master và slave. Tín hiệu xung nhịp luôn được điều khiển bởi master.
- Hoạt động ở chế độ bán song công: Có khả năng truyền và nhận, nhưng trong 1 thời điểm chỉ làm 1 nhiệm vụ (OR)
- Một Master có thể giao tiếp với nhiều Slave
- Sử dụng 2 dây giao tiếp:
	- SDA (Serial Data) - đường truyền cho master và slave để gửi và nhận dữ liệu.
	- SCL (Serial Clock) - đường mang tín hiệu xung nhịp.

![image](https://github.com/user-attachments/assets/d22a3b9b-f705-4555-b1f4-4685629aa19a)

- I2C nằm ở chế độ open drain: khi I2C kiểm soát đường dây sẽ hạ đường dây xuống mức 0, không kiểm soát thì sẽ thả trôi-> MCU và các thiết bị khác không hiểu nên phải có trở treo 5v để biến thành mức 1
- Nhiều master với nhiều slave: Nhiều master có thể được kết nối với một slave hoặc nhiều slave. Sự cố với nhiều master trong cùng một hệ thống xảy ra khi hai master cố gắng gửi hoặc nhận dữ liệu cùng một lúc qua đường SDA. Để giải quyết vấn đề này, mỗi master cần phải phát hiện xem đường SDA thấp hay cao trước khi truyền tin nhắn. Nếu đường SDA thấp, điều này có nghĩa là một master khác có quyền điều khiển bus và master đó phải đợi để gửi tin nhắn. Nếu đường SDA cao thì có thể truyền tin nhắn an toàn. Để kết nối nhiều master với nhiều slave.
#### 2.2. Khung truyền I2C

![image](https://github.com/user-attachments/assets/12bf27aa-0940-42a5-a94b-d50d21a63584)

- Điều kiện khởi động: Đường SDA chuyển từ mức điện áp cao xuống mức điện áp thấp trước khi đường SCL chuyển từ mức cao xuống mức thấp.
- Khung địa chỉ: Một chuỗi 7 hoặc 10 bit duy nhất cho mỗi slave để xác định slave khi master muốn giao tiếp với nó. Master gửi địa chỉ của slave mà nó muốn giao tiếp với mọi slave được kết nối với nó. Sau đó, mỗi slave sẽ so sánh địa chỉ được gửi từ master với địa chỉ của chính nó. Nếu địa chỉ phù hợp, nó sẽ gửi lại một bit ACK điện áp thấp cho master. Nếu địa chỉ không khớp, slave không làm gì cả và đường SDA vẫn ở mức cao.
- Bit Đọc / Ghi: Một bit duy nhất chỉ định master đang gửi dữ liệu đến slave (mức điện áp thấp) hay yêu cầu dữ liệu từ nó (mức điện áp cao).
- Bit ACK / NACK: Mỗi khung trong một tin nhắn được theo sau bởi một bit xác nhận / không xác nhận. Nếu một khung địa chỉ hoặc khung dữ liệu được nhận thành công, một bit ACK sẽ được trả lại cho thiết bị gửi từ thiết bị nhận.
- Khung dữ liệu: Sau khi master phát hiện bit ACK từ slave, khung dữ liệu đầu tiên đã sẵn sàng được gửi. Khung dữ liệu luôn có độ dài 8 bit và được gửi với bit quan trọng nhất trước. Mỗi khung dữ liệu ngay sau đó là một bit ACK / NACK để xác minh rằng khung đã được nhận thành công. Bit ACK phải được nhận bởi master hoặc slave (tùy thuộc vào cái nào đang gửi dữ liệu) trước khi khung dữ liệu tiếp theo có thể được gửi.
- Điều kiện dừng: Đường SDA chuyển từ mức điện áp thấp sang mức điện áp cao sau khi đường SCL chuyển từ mức thấp lên mức cao.
### 3. UART
#### 3.1. Đặc điểm
- UART (Universal Asynchronous Receiver-Transmitter – Bộ truyền nhận dữ liệu không đồng bộ) là một giao thức truyền thông phần cứng dùng giao tiếp nối tiếp không đồng bộ và có thể cấu hình được tốc độ
- Chuẩn giao tiếp nối tiếp
- Không đồng bộ
- Chỉ 2 thiết bị giao tiếp với nhau
- Hoạt động ở chế độ song công
- Sử dụng 2 dây giao tiếp:
	- Tx (Transmit): Chân truyền dữ liệu 
	- Rx (Receive): Chân nhận dữ liệu

![image](https://github.com/user-attachments/assets/bd10772b-fc45-402e-a545-6410f3dea615)

- Dữ liệu được truyền và nhận qua các đường truyền này dưới dạng các khung dữ liệu (data frame) có cấu trúc chuẩn, với một bit bắt đầu (start bit), một số bit dữ liệu (data bits), một bit kiểm tra chẵn lẻ (parity bit) và một hoặc nhiều bit dừng (stop bit).
- Vấn đề 2 MCU khác nhau -> phải đồng bộ 1 bit phải truyền bao nhiêu giây
	```c
	  Baudrate = số bits truyền được/1s. 
	
	Ví dụ: baudrate = 9600
	Tức là:	Gửi 9600 bits trong	        1000ms
		Gửi 1 bits trong 	          ? ms 	=> 0.10417ms
	
	=> Timer (0 -> 0.10417 ms)
#### 3.2. Khung truyền UART

![image](https://github.com/user-attachments/assets/e357fc1e-7310-44e7-a311-1fcae4f03559)

- Quá trình truyền dữ liệu Uart sẽ diễn ra dưới dạng các gói dữ liệu này, bắt đầu bằng 1 bit bắt đầu, 2 chân Tx, Rx đường mức cao được kéo dần xuống thấp. Sau bit bắt đầu là 5 – 9 bit dữ liệu truyền trong khung dữ liệu của gói, theo sau là bit chẵn lẻ tùy chọn để nhằm xác minh việc truyền dữ liệu thích hợp. Sau cùng, 1 hoặc nhiều bit dừng sẽ được truyền ở nơi đường đặt tại mức cao. Vậy là sẽ kết thúc việc truyền đi một gói dữ liệu
- Bit parity là bit quy luật chẵn lẽ để check các bit data trước đó. 2 con MCU sẽ thống nhất quy luật chẵn lẻ của bit parity. Giá trị bit partity cuar bên truyền sẽ phụ thuộc vào số bit 1 trong bit data. Còn bit parity của bên nhận sẽ kiểm tra bit parity của bên truyền của khớp vơis mình không.
</details>

<details><summary>LESSON 5: SPI SOFTWARE & SPI HARDWARE</summary>
    <p>
        
## LESSON 5:SPI SOFTWARE & SPI HARDWARE
- Trên mỗi dòng vi điều khiển khác nhau module SPI sẽ được tích hợp, điều khiển bởi các thanh ghi,phần cứng, IO khác nhau, đấy gọi là SPI cứng (hardware SPI). Như vậy bản chất chuẩn truyền thông SPI giống nhau trên mỗi chip nhưng lại được cài đặt và sử dụng không giống nhau. Điều này gây thêm khó khăn cho người sử dụng khi bạn bắt đầu tìm hiểu một dòng vi điều khiển mới, bạn sẽ phải nhớ các chân MISO, SS, MOSI, SCK mỗi chip khác nhau, nhớ các thanh ghi, các chế độ hoạt động và cách cài đặt trên các dòng vi điều khiển khác nhau. 
- vì vậy SPI Software được dùng để thực hiện giao tiếp SPI bằng phần mềm, tức là điều khiển trực tiếp các chân GPIO để mô phỏng giao thức SPI.
- Ưu điểm:
	- Linh hoạt: Có thể sử dụng bất kỳ chân GPIO nào, không phụ thuộc vào phần cứng hỗ trợ SPI.
	- Dễ triển khai: Chỉ cần thao tác mức logic trên chân GPIO.
	- Tương thích với nhiều vi điều khiển: Không cần module SPI chuyên dụng.
- SPI dùng 4 chân để truyền nhận, gồm MISO, MOSI, CS và SCK.
![image](https://github.com/user-attachments/assets/d916eaa1-487a-458f-af01-1d894e0472bb)
- MISO: (Master In Slave Out) Chân nhận tín hiệu của Master nối với chân truyền của Slave, vì vậy được cấu hình Input ở Master và Output ở Slave.
- MOSI: (Master Out SLave In) Ngược lại với MISO, cấu hình Input cho Slave và Output cho Master.
- SCK: (Clock) Chân truyền tín hiệu xung đồng bộ từ Master cho Slave, được cấu hình Output cho Master và Input cho Slave.
- CS: (Chip Select) Chân gửi tín hiệu chọn Slave của Master tới các Slave. Cấu hình Output cho Master và Input cho Slave. Có thể có nhiều CS để Master điều khiển nhiều Slave.

### SPI SOFTWARE
#### Cấu hình xung clock
![image](https://github.com/user-attachments/assets/7a393e88-7804-4fff-8af7-84a3b7896477)
- Xác nhận chọn các chân và cấu hình GPIO
	```c
	 #define SPI_SCK_Pin GPIO_Pin_0 // dinhnghia 4 chan su dung spi
	#define SPI_MISO_Pin GPIO_Pin_1
	#define SPI_MOSI_Pin GPIO_Pin_2
	#define SPI_CS_Pin GPIO_Pin_3
	#define SPI_GPIO GPIOA
	#define SPI_RCC RCC_APB2Periph_GPIOA
- Sử dụng 4 chân của GPIOA để giao tiếp SPI, cấp clock cho 4 chân này và định nghĩa tên từng chân theo SPI để dễ quản lí
	```c
 	void RCC_config(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
	}
- dùng hàm RCC_Config để cấp clock cho TIM2(cổng APB1,dùng để cấp timer tạo hàm delay()) và GPIOA(cổng APB2, dùng để giao tiếp spi 4 chân)
#### Cấu hình GPIO 
- Mô phỏng xung clock bằng cách cho chân SCK lên mức 1, delay(), rồi xuống lại mức 0
	```c
 	void Clock(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
	delay_ms(4);
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	delay_ms(4);
	}
 - Khởi tạo giá trị đầu cho các chân SPI, SCK chưa có xung nên sẽ là bit 0 (CPOL = 0) và CS ở mức 1 (chưa chọn Slave). Chân MOSI mức gì cũng được và chân MISO master không thể điều khiển.
	```c
 	void SPISetup(void){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin,  Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin,   Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);//Muc gi cung duoc
 	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
	}
 
##### Master
- cấu hình:
	```c
 	 void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin| SPI_MOSI_Pin| SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	}
- Đây là cấu hình cho Master nên sẽ có ba chân GPIOA ngõ ra là SCK, MOSI và CS (mode Output pushpull)
- tốc độ 50MHZ
- Chân nhận tín hiệu của Master nối với chân truyền của Slave, vì vậy được cấu hình Input ở Master
##### Slave
- cấu hình:
	```c
 	 void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	}
-  3 chân SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin của slave là input, nhận tín hiệu của master
-  chân SPI_MISO_Pin là output truyền dữ liệu của slave cho master
#### Hàm truyền
##### Master
- Hàm truyền dữ liệu của master
	```c
 	void SPI_Master_Transmit(uint8_t u8Data){
	uint8_t u8Mask = 0x80;					
	uint8_t tempData; 
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_RESET); 
	delay_ms(1);
	for(int i=0; i<8; i++){
		tempData = u8Data & u8Mask; 
		if(tempData){
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_SET);
			delay_ms(1);
		} else{
			GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
			delay_ms(1);
		}
		u8Data=u8Data<<1; 
		Clock();
	}
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	delay_ms(1);
	}

- Tạo một mặt nạ có giá trị 0b10000000 và tạo một biến đệm để xử lý bit
- Đầu tiên sẽ phải kéo chân CS của Slave muốn giao tiếp xuống 0 và đợi 1ms để đảm bảo chân CS xuống được mức 0 và Slave nhận được tin hiệu.
- Sau đó gửi 8 bit bằng cách dịch từng bit của u8Data bằng cách dùng (and) với u8Mask để tìm ra bit đang truyền là bit 0 hay 1 và gán vào tempData (0&1 =0; 1&1=1)
- Cuối cùng là kiểm tra giá trị của tempData để đặt chân MOSI là cao hay thấp, tương ứng với bit 1 và 0.	
- sau đó dịch 1 bit để tiếp tục cho lần gửi bit tiếp theo
	```c
 	uint8_t DataTrans[] = {1,7,12,17,89};//Du lieu duoc truyen di
	int main(){
	    RCC_config();
	    TIMER_config();
	    GPIO_Config();
	    SPISetup();
	    while(1){	
				for(int i=0; i<5; i++){
					SPI_Master_Transmit(DataTrans[i]);
					delay_ms(1000);
				}
			}
	}





