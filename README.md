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
#### Hàm truyền (Master)
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
- Tại hàm main() sẽ gọi lại các hàm cấu hình GPIO và Timer. Sau đó tạo một hàm while(1) để gửi tuần tự các giá trị của mảng DataTrans
#### Hàm nhận (Slave)
- hàm nhận dữ liệu của Slave
  	```c
	uint8_t SPI_Slave_Receive(void){
		uint8_t dataReceive =0x00;   
		uint8_t temp = 0x00, i=0;
		while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
		for(i=0; i<8;i++){ 
			if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
				while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin))
					temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
				
				dataReceive=dataReceive<<1;
				dataReceive=dataReceive|temp;
	    }
			while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
		}
		return dataReceive;
	}

- Giá trị nhận được ở Slave cũng là 8 bit.
- Đầu tiên tạo một biến để nhận dữ liệu dataReceive và một biến đệm temp.
- Chờ cho đến khi chân CS được kéo xuống 0
- Chờ đến khi có xung Clock (Có dữ liệu được truyền).
- Dữ liệu nhận được sẽ gán vào biến temp và dịch vào dataReceive
	```c
	uint8_t Num_Receive;
	int main(){
	    RCC_config();
	    TIMER_config();
			GPIO_Config();
			SPISetup();
	    TIM_SetCounter(TIM2,0); //Set up gia tri trong thanh ghi dem
	    while(1){	
				if(!(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin))){
					for(int i=0; i<5; i++){
						Num_Receive = SPI_Slave_Receive();
					}
				}
			}
	}
- Tại hàm main sẽ liên tục kiểm tra chân CS và nhận 5 bit dữ liệu từ Master.
### SPI HARDWARE
#### Định nghĩa các chân SPI
- STM32 cấu hình sẵn các chân dành cho chức năng SPI.
  ![image](https://github.com/user-attachments/assets/aeb11ec1-5ab9-4a86-80f9-8965b5e98241)
  	```c
	#define SPI1_NSS 	GPIO_Pin_4
	#define SPI1_SCK	GPIO_Pin_5
	#define SPI1_MISO 	GPIO_Pin_6
	#define SPI1_MOSI 	GPIO_Pin_7
	#define SPI1_GPIO 	GPIOA
#### Cấu hình xung Clock và chân GPIO cho SPI 
- cấu hình cấp xung và GPIO
	```c
 	void RCC_Config() 
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	}
	void GPIO_Config(){
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = SPI1_NSS | SPI1_SCK | SPI1_MISO | SPI1_MOSI;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
		GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
	}
#### Cấu hình SPI 
- cấu hình SPI cho hardware
	```c
 	void SPI_Config(){
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
	}
- Các tham số cấu hình:

	- SPI_Mode: Quy định chế độ hoạt động của thiết bị SPI.
	- SPI_Direction: Quy định kiểu truyền của thiết bị.
	- SPI_BaudRatePrescaler: Hệ số chia clock cấp cho Module SPI.
	- SPI_CPOL: Cấu hình giá trị SCK khi chế độ nghỉ: SPI_CPOL_Low (mức 0), SPI_CPOL_High (mức 1)
	- SPI_CPHA: khoảnh khắc lấy mẫu dữ liệu từ Slave theo SCL: SPI_CPHA_1Edge: (ở cạnh xung đầu tiên), SPI_CPHA_2Edge: (ở cạnh xung thứ hai).
	- SPI_DataSize: Cấu hình số bit truyền. 8 hoặc 16 bit.
	- SPI_FirstBit: Cấu hình chiều truyền của các bit là MSB hay LSB.
	- SPI_CRCPolynomial: Cấu hình số bit CheckSum cho SPI.
	- SPI_NSS: Cấu hình chân SS là điều khiển bằng thiết bị hay phần mềm.
 - Đối với Slave các thông số cấu hình giống Master nhưng khác ở SPI_Mode = SPI_Mode_Slave
 #### Hàm truyền nhận của SPI
 - Hàm truyền của Master:
 	```c
	 void SPI_Send1Byte(uint8_t data){
	    GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_RESET);
	   
	    SPI_I2S_SendData(SPI1, data);
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==0);
	   
	    GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_SET);
	}
- Hàm nhận của Slave
  	```c
	uint8_t SPI_Receive1Byte(void){
	    uint8_t temp;
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);
	    temp = (uint8_t)SPI_I2S_ReceiveData(SPI1);
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0);
	    return temp;
	}
- Một số hàm để làm việc với SPI:
	- HàmSPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data), tùy vào cấu hình datasize là 8 hay 16 bit sẽ truyền đi 8 hoặc 16 bit dữ liệu. Hàm nhận 2 tham số là bộ SPI sử dụng và data cần truyền.
	- Hàm SPI_I2S_ReceiveData(SPI_TypeDef* SPIx) trả về giá trị đọc được trên SPIx. Hàm trả về 8 hoặc 16 bit data.
	- Hàm SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG) trả về giá trị 1 cờ trong thanh ghi của SPI. Các cờ thường được dùng:
		- SPI_I2S_FLAG_TXE: Cờ báo truyền(đã truyền xong dữ liệu = trống), cờ này sẽ set lên 1 khi truyền xong data trong buffer.
		- SPI_I2S_FLAG_RXNE: Cờ báo nhận(có dữ liệu tới), cờ này set lên 1 khi nhận xong data.
		- SPI_I2S_FLAG_BSY: Cờ báo bận,set lên 1 khi SPI đang bận truyền nhận.
- Hàm truyền và nhận của master
- Đầu tiên trước khi truyền phải kéo chân CS của Slave xuống mức thấp, sau đó gọi hàm SPI_I2S_SendData(SPI1, data);, có hai tham số là bộ SPI và dữ liệu truyền đi để truyền đi 8 bit data
- Sau đó chờ đến khi cờ TXE được kéo lên 1 (truyền xong).
- Theo lý thuyết, Slave cũng sẽ gửi lại data cho Master nên ta sẽ chờ cờ RXNE được kéo lên 1 (nhận xong) và đọc dữ liệu được nhận trong thanh ghi DR của SPI1 bằng hàm SPI_I2S_ReceiveData(SPI1);
- Sau khi đã hoàn thành, đặt lại chân CS lên 1 để bỏ chọn Slave.
  	```c
   	uint8_t SPI_Send1Byte(uint8_t data){
	uint8_t received_data;
	GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_RESET); 
 
	SPI_I2S_SendData(SPI1, data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)==0); 
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
   received_data = SPI_I2S_ReceiveData(SPI1);
	
	GPIO_WriteBit(SPI1_GPIO, SPI1_NSS, Bit_SET); 
	return received_data; }
   - Hàm truyền và nhận của slave
  	```c
	uint8_t SPI_Receive1Byte(uint8_t data_to_send_back){
	    uint8_t temp;
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==1);//Co bao nhan 	SPI_I2S_FLAG_BSY = 1 khi SPI dang ban, Cho` den khi SPI ranh?
	    temp = (uint8_t)SPI_I2S_ReceiveData(SPI1); // Tra ve gia tri doc duoc tren SPI1
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==0); //cho` den khi nhan xong data SPI_I2S_FLAG_RXNE = 1
	    
			SPI_I2S_SendData(SPI1, data_to_send_back); // G?i d? li?u tr? l?i
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		return temp;}
- Đọc chân NSS và chờ đến khi chân được kéo xuống 0
- Sau đó, chờ đến khi SPI1 rảnh bằng cách đọc cờ SPI_I2S_FLAG_BSY. Bằng 0 thì rảnh
- Tiếp theo đó, đọc dữ liệu nhận được từ Master thông qua hàm SPI_I2S_ReceiveData(SPI1);
- Đọc cờ RXNE cho đến khi nhận xong. Sau đó gửi lại dữ liệu đến Master bằng hàm SPI_I2S_SendData(SPI1, data_to_send_back);
- Sau đó chờ đến khi gửi xong TXE == 1 thì kết thúc hàm
#### Hàm main()

	```c
	uint8_t Num_Receive;
	uint8_t k[5] = {12, 13, 14, 15, 16};
	int main(){
	    RCC_config();
			GPIO_Config();
			SPI_config();
	    while(1){   
				while(GPIO_ReadInputDataBit(SPI1_GPIO, SPI1_NSS));
	       if(GPIO_ReadInputDataBit(SPI1_GPIO, SPI1_NSS)==0){
	          for(int i = 0; i<5; i++){
	            Num_Receive = SPI_Receive1Byte(k[i]);
	          }
	       }
			}
	}

</details>

<details><summary>LESSON 6: I2C SOFTWARE & I2C HARDWARE</summary>
    <p>


## LESSON 6: I2C SOFTWARE & I2C HARDWARE
- I2C chỉ sử dụng hai dây để truyền dữ liệu giữa các thiết bị:
	- SDA (Serial Data) - đường truyền cho master và slave để gửi và nhận dữ liệu.
	- SCL (Serial Clock) - đường mang tín hiệu xung nhịp.
   ![image](https://github.com/user-attachments/assets/24a2e4a6-578f-4913-913c-99a844e0a029)

### 1. I2C SOFTWARE
#### 1.1. Xác định các chân I2C
- Xác định chân SCL là PB6 và SDA là PB7. Sau đó cấp xưng cho GPIOB và Timer2	
	```c
 	#define I2C_SCL 	GPIO_Pin_6
	#define I2C_SDA		GPIO_Pin_7
	#define I2C_GPIO 	GPIOB
	
	void RCC_Config(){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
#### 1.2. Cấu hình GPIO
- Cấu hình SDA và SCL là Output Open-Drain (mức 0 và lơ lửng): Có khả năng kéo chân xuống mức 0 và dùng điện trở để kéo lên 1 (để tránh tín hiệu lơ lửng gây nhiễu, loạn). Gọi hàm Init để gán thông số vào các chân I2C của GPIOB
	```c
 	void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = I2C_SDA| I2C_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
	}
 #### 1.3. Cấu hình mô phỏng truyền nhận
 - Tạo các Macro để sử dụng thuận tiện hơn trong việc ghi các chân SDA, SCL và đọc giá trị tại chân SDA.
  	```c 
	   #define WRITE_SDA_0 	GPIO_ResetBits(I2C_GPIO, I2C_SDA)
	#define WRITE_SDA_1 	GPIO_SetBits(I2C_GPIO, I2C_SDA)
	#define WRITE_SCL_0 	GPIO_ResetBits(I2C_GPIO, I2C_SCL)
	#define WRITE_SCL_1 	GPIO_SetBits(I2C_GPIO, I2C_SCL)
	#define READ_SDA_VAL 	GPIO_ReadInputDataBit(I2C_GPIO, I2C_SDA)
 - Tạo hai Enum chứa giá trị được cấu hình của status (truyền thành công?) và ACK_bit(Giá trị của ACK)
	```c
 	typedef enum {
	NOT_OK, OK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
	} status;
	
	typedef enum {
		NACK, ACK //status la bien co hai gia tri: 0:NOT_OK va 1:OK
	} ACK_Bit;
- Cấu hình cho các chân SPI khi chưa truyền dữ liệu thì SDA và SCL đều kéo lên mức 1
	```c
 	void I2C_Config(void){
	WRITE_SDA_1; 
	delay_us(1);
	WRITE_SCL_1;
	delay_us(1);
	}
 - Hàm Start: Gán lại hai chân lên 1 trước khi truyền. Sau đó bắt đầu tín hiệu Start bằng cách kéo SDA xuống 0 trước sau đó kéo SCL xuống 0.
	```c
 	void I2C_Start(){
	//Dam bao rang SDA va SCL = 1 truoc khi truyen data
	WRITE_SDA_1;
	delay_us(1);	
	WRITE_SCL_1;  	
	delay_us(3);
	//SDA keo xuong 0 truoc SCL	
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_0;
	delay_us(3);
	}
 - Tương tự hàm Stop sẽ đảm bảo SDA = 0 trước, kéo chân SCL lên 1 trước sau đó kéo SDA lên 1
	```c
 	void I2C_Stop(){
	WRITE_SDA_0;
	delay_us(3);
	WRITE_SCL_1; 	//SCL set to 1 before SDA.
	delay_us(3);
	WRITE_SDA_1;
	delay_us(3);
	
 	}
 - Hàm gửi dữ liệu đến Slave của Master
	- Đầu tiên, tạo mặt nạ để and với dữ liệu để tìm ra bit truyền là 1 hay 0
	- Sau đó truyền dữ liệu qua SDA với giá trị vừa tìm được kèm theo một xung Clock được tạo bằng cách kéo và hạ chân SCL trong một khoảng delay(5us).
	- Dịch một bit và tiếp tục and với mặt nạ để tiếp tục truyền.
	- Sau khi truyền 8 bit dữ liệu Master sẽ chờ để nhận ACK. Mặc định SDA được kéo lên 1.
	- Tạo xung để nhận ACK từ Slave. Nếu nhận SDA = 0 thì nhận được ACK, nếu SDA = 1 thì không nhận được ACK.
	- Hàm trả về trạng thái của ACK nhận được.
 	```c
  	status I2C_Write(uint8_t u8Data){	

	status stRet;
	for(int i=0; i<8; i++){	
		if (u8Data & 0x80)// 0b10000000
		{
			WRITE_SDA_1;
		} else {
			WRITE_SDA_0;
		}	
		delay_us(3);
		WRITE_SCL_1;
		delay_us(5);
		WRITE_SCL_0;
		delay_us(2);
		
		u8Data <<= 1;
	}
	WRITE_SDA_1;					
	delay_us(3);

	WRITE_SCL_1;
	delay_us(3);

	if (READ_SDA_VAL) {	
		stRet = NOT_OK;				
	} else {
		stRet = OK;					
	}

	delay_us(2);
	WRITE_SCL_0;
	delay_us(5);
	
	return stRet;
	}
- Hàm đọc dữ liệu từ Slave
	- Tham số truyền vào là ACK_bit để xác nhận muốn đọc tiếp hay dừng lại
	- Khởi tạo SDA = 1
	- Tạo xung Clock, đồng thời đọc dữ liệu ở chân SDA và dịch vào dữ liệu 8 bit nhận được
	- Sau đó kiểm tra tham số _ACK để xác định có truyền lại ACK hay không kèm theo xung Clock
	- Dựa vào khung truyền của từng Slave mà sẽ có những cấu trúc truyền khác nhau
 	```c
  	uint8_t I2C_Read(ACK_Bit _ACK){	
	uint8_t i;						
	uint8_t u8Ret = 0x00;
	//Dam bao SDA pull_up truoc khi nhan
	WRITE_SDA_1;
	delay_us(3);	
	//Tao 8 clock va doc du lieu ghi vao u8Ret
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		WRITE_SCL_1;
		delay_us(3);
		if (READ_SDA_VAL) {//Doc du lieu chan ACK
			u8Ret |= 0x01;
		}
		delay_us(2);
		WRITE_SCL_0;
		delay_us(5);
	}
	if (_ACK) {	//Neu tham so truyen vao la ACK thi truyen ACK di 
		WRITE_SDA_0;
	} else {
		WRITE_SDA_1;
	}
	delay_us(3);
	//Tao xung de truyen ACK di
	WRITE_SCL_1;
	delay_us(5);
	WRITE_SCL_0;
	delay_us(5);

	return u8Ret;
	}
### 2. I2C HARDWARE
- Các bước thực hiện

	- Xác định các chân GPIO của I2C
	- Cấu hình GPIO
	- Cấu hình I2C
#### 2.1. Xác định các chân GPIO
- Xác định các chân GPIO
	```c
	#define I2C_SCL 	GPIO_Pin_6
	#define I2C_SDA		GPIO_Pin_7
	#define I2C_GPIO 	GPIOB
	#define DS1307_ADDRESS 0x50
	
	void RCC_Config(void) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_I2C1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
}
- Trên STM32F103C8T6 có hai bộ I2C1 và I2C2. Với SPI1 sẽ sử dụng hai chân PB6 và PB7. Và ta sẽ cấp xung cho Timer và I2C1, đồng thời cấp cho GPIOB để GPIO hoạt động.

#### 2.2. Cấu hình GPIO
	
	void GPIO_Config(void) {
	    GPIO_InitTypeDef GPIO_InitStructure;
	
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
- Tương tự với SPI, các GPIO cũng sử dụng mode AF để sử dụng cho chế độ thay thế. Nhưng với I2C sẽ sử dụng Open-Drain

#### 2.3. Cấu hình I2C
	
 
	void I2C_Config(void) {
		I2C_InitTypeDef I2C_InitStruct;
		I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStruct.I2C_ClockSpeed = 400000;
		I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
		I2C_Init(I2C1, &I2C_InitStruct);
		I2C_Cmd(I2C1, ENABLE);
		}
- Tương tự các ngoại vi khác, các tham số I2C được cấu hình trong Struct I2C_InitTypeDef:
	- I2C_Mode: Cấu hình chế độ hoạt động cho I2C:
		- I2C_Mode_I2C: Chế độ I2C FM(Fast Mode);
		- I2C_Mode_SMBusDevice&I2C_Mode_SMBusHost: Chế độ SM(Slow Mode).
- I2C_ClockSpeed: Cấu hình clock cho I2C, tối đa 100khz với SM và 400khz ở FM.
- I2C_DutyCycle: Cấu hình chu kì nhiệm vụ của xung:
		- I2C_DutyCycle_2: Thời gian xung thấp/xung cao = 2;
		- I2C_DutyCycle_16_9: Thời gian xung thấp/xung cao =16/9; Ví dụ:
		- I2C_DutyCycle_2: tLow/tHigh = 2 => tLow = 2tHigh 100000khz, 1xung 10us 6.66us low, 3.33 high
		- I2C_DutyCycle_16_9: tLow/tHigh = 16/9 => 9tLow = 16tHigh.
- I2C_OwnAddress1: Cấu hình địa chỉ slave.
- I2C_Ack: Cấu hình ACK, có sử dụng ACK hay không.
- I2C_AcknowledgedAddress: Cấu hình số bit địa chỉ. 7 hoặc 10 bit
#### 2.4. Các hàm truyện nhận I2C

- Hàm I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction), gửi đi 7 bit address để xác định slave cần giao tiếp. Hướng truyền được xác định bởi I2C_Direction để thêm bit RW.
- Hàm I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data) gửi đi 8 bit data.
- Hàm I2C_ReceiveData(I2C_TypeDef* I2Cx) trả về 8 bit data.
- Hàm I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT) trả về kết quả kiểm tra I2C_EVENT tương ứng:
	- I2C_EVENT_MASTER_MODE_SELECT: Đợi Bus I2C về chế độ rảnh.
	- I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu nhận của Master.
	- I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: Đợi xác nhận của Slave với yêu cầu ghi của Master.
	- I2C_EVENT_MASTER_BYTE_TRANSMITTED: Đợi truyền xong 1 byte data từ Master.
	- I2C_EVENT_MASTER_BYTE_RECEIVED: Đợi Master nhận đủ 1 byte data.
 	```c
	I2C_GenerateSTART(I2C1, ENABLE);
	 //Waiting for flag
	 while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, 0x44, I2C_Direction_Transmitter);
	//And check the transmitting
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
- Hàm truyền địa chỉ chọn Slave:
	- Tạo tín hiệu bắt đầu bằng hàm I2C_GenerateSTART
	- Đợi ACK từ Slave
	- Và truyền 7 bit địa chỉ đến các Slave trong mạng để chọn Slave cần giao tiếp
	- Chờ đến khi truyền xong và có xác nhận từ Slave
 	```c
	void Send_I2C_Data(uint8_t data)
	{
		I2C_SendData(I2C1, data);
		// wait for the data trasnmitted flag
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
- Hàm gửi 8 bit dữ liệu: Gửi data qua I2C1 và chờ đến khi hoàn tất truyền
	```c
	uint8_t Read_I2C_Data(){
		
		uint8_t data = I2C_ReceiveData(I2C1);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		return data;
	}
- Hàm nhận 8 bit dữ liệu: Sử dụng các hàm trên theo các câu trúc khác nhau để giao tiếp với slave
</details>
<details><summary>LESSON 7: UART</summary>
    <p>
        
## LESSON 7: UART SOFTWARE & HARDWARE
### 1. UART SOFTWARE
#### 1.1. Xác định chân và cấu hình GPIO
![image](https://github.com/user-attachments/assets/8779ba7d-5d3f-49b9-8c46-4f1b5651ef02)

- Sử dụng các GPIO để mô phỏng quá trình truyền nhận của UART
- Có hai bước để thực hiện:
	- Xác định các chân UART
	- Cấu hình GPIO
   	```c
	#define TX_Pin GPIO_Pin_0
	#define RX_Pin GPIO_Pin_1
	#define UART_GPIO GPIOA
	#define time_duration 104

	void RCC_Config(){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
- Chúng ta sẽ sử dụng A0 làm TX và A1 làm RX và cấp xung cho chúng.
	```c
	void delay_us(uint16_t timedelay){
	    TIM_SetCounter(TIM2, 0);
	    while(TIM_GetCounter(TIM2)<timedelay){}
	}
	
	void delay_s(uint32_t timedelay){
	    TIM_SetCounter(TIM2, 0);
				for(int i = 0; i < timedelay*1000000; i++){
					delay_us(1);
				}
	}
- Tạo hai hàm delay sử dụng Tỉmer2 đã được đề cập từ bài trước.

	void clock(){
		delay_us(time_duration);
	}
- Một hàm tạo delay, là khoảng thời gian giữa hai lần truyền dữ liệu để hai bên đồng bộ.
- time_duration là khoảng thời gian delay khi chọn Baudrate = 9600.
![image](https://github.com/user-attachments/assets/0c46b3e6-e939-41c9-8de0-e7d5a995878f)

- BaudRate = 9600
- Tức truyền được 9600bits/s = 9600bits/1000ms
- Nghĩa là 1 bit truyền đi mất khoảng 0.10467ms
- Vì thế time_duration = 104 để xấp xỉ.
  	```C
	void GPIO_Config(){
		GPIO_InitTypeDef GPIOInitStruct;
		GPIOInitStruct.GPIO_Pin = RX_Pin;
		GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIOInitStruct);
		//
		GPIOInitStruct.GPIO_Pin = TX_Pin;
		GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIOInitStruct);
	}
- Ta sẽ cấu hình RX kiểu Input Floating để làm ngõ vào và TX Output Push-Pull để tạo ngõ ra.
	```c
	void UART_Config(){
		GPIO_SetBits(UART_GPIO, TX_Pin);
		delay_us(1);
	}
- Và khi chưa truyền dữ liệu thì cả hai chân được nối mức cao.

#### 1.2. Hàm mô phỏng gửi dữ liệu kí tự

	```c
	void UARTSoftware_Transmit(char c) {
	    // Start bit
	    GPIO_ResetBits(GPIOA, TX_Pin); // Tao dieu kien bat dau
	    clock();
	
	    // Truyen các bit du lieu (LSB truoc)
	    for (int i = 0; i < 8; i++) {
	        if (c & (1 << i)) {
	            GPIO_SetBits(GPIOA, TX_Pin);
	        } else {
	            GPIO_ResetBits(GPIOA, TX_Pin);
	        }
	        clock();
	    }
	
	    // Stop bit
	    GPIO_SetBits(GPIOA, TX_Pin);
	    clock();
	}
- Trước khi gửi, phải kéo chân TX xuống mức và delay một chu kỳ để bắt đầu tín hiệu Start
- Sau đó sẽ truyền lần lượt 8 bits dữ liệu bằng cách dịch và set chân TX theo giá trị vừa tìm và delay trong một khoảng chu kỳ
- Sau khi truyền xong 8 bits dữ liệu sẽ truyền bits Stop tức kéo chân TX lên mức 1 để kết thúc truyền nhận (với trường hợp không dùng Parity).
#### 1.3. Hàm mô phỏng nhận ký tự

	```c
	char UARTSoftware_Receive() {
	    char c = 0;
			// Start bit
			while (GPIO_ReadInputDataBit(GPIOA, RX_Pin) == 1);
	
			delay_us(time_duration + time_duration / 2);
	
			for (int i = 0; i < 8; i++) {
					if (GPIO_ReadInputDataBit(GPIOA, RX_Pin)) {
							c |= (1 << i);
					}
					clock(); 
			}
	
			// Đợi Stop bit
			delay_us(time_duration / 2);
			
			return c;
	}
- Tại bên nhận sẽ chờ cho đến khi nhận được tín hiệu Start
- Sau đó sẽ delay 1.5 chu kỳ để khi đọc dữ liệu, sẽ đọc tại vị trí giữa của một bit, nơi tín hiệu đã ổn định
- Lần lượt nhận 8 bits dữ liệu tại chân RX và dịch vào c
- Delay nửa chu kỳ cuối để đợi Stop bit và trả về ký tự c là dữ liệu nhận được
  	```c
	char data[9] = {'V', 'A', 'N', 'T', 'U', 'H', 'A', 'L', 'A'};
	int main(){
		RCC_Config();
		GPIO_Config();
		TIMER_config();
		UART_Config();
		for (int i = 0; i<9; i++){
				UARTSoftware_Transmit(data[i]);
			delay_s(1);
		}
			UARTSoftware_Transmit('\n');
			
		while(1){
			UARTSoftware_Transmit(UARTSoftware_Receive());
		}	
	}
- Hàm test sẽ lần lượt gửi 9 ký tự qua UART và sau đó sẽ chờ để nhận dữ liệu nhận về của UART.

### 2. UART Hardware
- Sử dụng UART được tích hợp trên phần cứng của vi điều khiển.
- Các bước thực hiện

	- Xác định các chân GPIO của UART
	- Cấu hình GPIO
	- Cấu hình UART
- Ở bài này chúng ta sẽ sử dụng USART1 có hai chân PA9 - TX và PA10 - RX
- Để sử dụng UART, ta phải thêm thư viện  "stm32f10x_usart.h"

#### 2.1. Cấu hình GPIO


 
void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_10; //Chan RX
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;// Neu 2 chan de AF thi bi ngan mach
	GPIO_Init(GPIOA, &GPIOInitStruct);
	
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_9; //Chan TX
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}
- Đầu tiên phải cấp xung GPIOA để sử dụng các chân và cấp thêm cho bộ USART1.

- Chân RX được cấu hình GPIO_Mode_IN_FLOATING và ngõ ra TX được cấu hình GPIO_Mode_AF_PP. Không cấu hình RX là AF_PP vì sẽ bị ngắn mạch hai chân RX và TX.

#### 2.2. Cấu hình UART


  
void UART_Config(void){
	USART_InitTypeDef UARTInitStruct;
	UARTInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //Cau hinh che do: ca truyen va nhan (song cong)
	UARTInitStruct.USART_BaudRate = 115200; //Cau hinh toc do bit
	UARTInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Cau hinh kiem soat luong truyen du lieu tranh viec tran bo dem
	UARTInitStruct.USART_WordLength = USART_WordLength_8b; //Truyen du lieu 8 hoac 9 bit
	UARTInitStruct.USART_Parity = USART_Parity_No;
	UARTInitStruct.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1, &UARTInitStruct);
	
	USART_Cmd(USART1, ENABLE);
	
}
- USART_Mode: Cấu hình Mode của vi điều khiển là nhận hay gửi hoặc cả hai khi sử dụng phép OR.

- USART_BaudRate: Cấu hình BaudRate để đồng bộ truyền nhận dữ liệu giữa hai vi điều khiển. Phổ biến nhất là 9600 và 115200.

- USART_HardwareFlowControl: Dùng để bật chế độ kiểm soát luồng truyền để tránh việc tràn bộ đệm. Có các giá trị sau:
	```c
	USART_HardwareFlowControl_None       
	USART_HardwareFlowControl_RTS    // Request to Send   
	USART_HardwareFlowControl_CTS    // Clear to Send
	USART_HardwareFlowControl_RTS_CTS
- RTS (Request To Send - Yêu cầu gửi)
	- Được điều khiển bởi thiết bị gửi dữ liệu.
	- Khi thiết bị gửi dữ liệu sẵn sàng để truyền, nó kéo RTS xuống mức thấp (Active Low).
	Khi bộ đệm của thiết bị nhận đầy, nó kéo RTS lên mức cao, báo cho thiết bị gửi tạm dừng truyền dữ liệu.
- CTS (Clear To Send - Cho phép gửi)

	- Được điều khiển bởi thiết bị nhận dữ liệu.
	- Khi bộ đệm của thiết bị nhận còn trống, nó kéo CTS xuống mức thấp, cho phép thiết bị gửi tiếp tục truyền dữ liệu.
	- Nếu bộ đệm đầy, thiết bị nhận kéo CTS lên mức cao, yêu cầu dừng gửi dữ liệu. Thiết bị A muốn gửi dữ liệu:
- Kiểm tra CTS từ thiết bị B:
	- Nếu CTS = LOW, thiết bị A có thể gửi dữ liệu.
	- Nếu CTS = HIGH, thiết bị A tạm dừng gửi để tránh tràn bộ đệm. Thiết bị B nhận dữ liệu:
	- Khi bộ đệm gần đầy, RTS = HIGH, báo hiệu thiết bị A tạm dừng truyền dữ liệu.
	- Khi bộ đệm có không gian trống, RTS = LOW, cho phép tiếp tục truyền.
- USART_WordLength: Khai báo dữ liệu truyền là 8 bits hoặc 9 bits

- USART_StopBits: Cấu hình số lượng Stopbit

  	- USART_StopBits_1
	- USART_StopBits_0_5 
	- USART_StopBits_2    
	- USART_StopBits_1_5    
	- USART_Parity: Cấu hình sử dụng Parity, có hai loại là chẵn hoặc lẽ, có thể không cần sử dụng
	```c
	#define USART_Parity_No
	#define USART_Parity_Even
	#define USART_Parity_Odd 
- Một số hàm được thiết lập sẵn trong UART
	- Hàm USART_SendData(USART_TypeDef* USARTx, uint16_t Data) truyền data từ UARTx. Data này đã được thêm bit chẵn/lẻ tùy cấu hình.
	- Hàm USART_ReceiveData(USART_TypeDef* USARTx) nhận data từ UARTx.
	- Hàm USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG) trả về trạng thái cờ USART_FLAG tương ứng:
			- USART_FLAG_TXE: Cờ truyền, set lên 1 nếu quá trình truyền hoàn tất.
			- USART_FLAG_RXNE: Cờ nhận, set lên 1 nếu quá trình nhận hoàn tất.
			- USART_FLAG_IDLE: Cờ báo đường truyền đang ở chế độ Idle.
			- USART_FLAG_PE: Cờ báo lỗi Parity.
- Hàm gửi một ký tự
	```c
	void UART_SendChar(USART_TypeDef *USARTx, char data){
		
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)==RESET); // Cho khi thanh ghi DR trong de chen du lieu moi de gui
		
		USART_SendData(USARTx, data);
		
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);//Cho den khi truyen thanh cong
		
		
	}
- Đợi đến khi thanh ghi DR trống đến thêm dữ liệu mới và gửi

- Gửi dữ liệu bằng hàm USART_SendData(USARTx, data);

- Sau đó chờ đến khi truyền thành công

- Hàm nhận một ký tự
	```c
	char UART_ReceiveChar(USART_TypeDef *USARTx){
		char tmp = 0x00;
		
		while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET);
		
		tmp = USART_ReceiveData(USARTx);
		
		return tmp;
	}
- Chờ đến khi nhận hoạt tất
- Nhận Data từ UARTx gán vào tmp.
- ví dụ:
	```c
	uint8_t DataTrans[] = {'V','A','N','T','U'};//Du lieu duoc truyen di
	int main() {
		RCC_Config();
		GPIO_Config();
		UART_Config();
		TIMER_config();	
		for(int i = 0; i<5; ++i){
				delay_ms(1998);
				UART_SendChar(USART1, DataTrans[i]);
				delay_ms(2);
			}
		UART_SendChar(USART1, '\n');
	
		while(1){
			UART_SendChar(USART1,UART_ReceiveChar(USART1));
		}
	}
- Sử dụng main() để test bằng cách gửi 5 bytes dữ liệu qua USART1 và sau đó chờ để nhận dữ liệu về.
</details>
<details><summary>LESSON 8: INTERRUPT</summary>
    <p>
        
## LESSON 8: INTERRUPT
### 1. Ngắt ngoài
#### 1.1. Định nghĩa
- External interrupt (EXTI) hay còn gọi là ngắt ngoài. Là 1 sự kiện ngắt xảy ra khi có tín hiệu can thiệp từ bên ngoài, từ phần cứng, người sử dụng hay ngoại vi,…
- Sơ đồ khối của các khối điều khiển ngắt ngoài
  ![image](https://github.com/user-attachments/assets/d62c10f3-51b3-42f1-94a5-ae416671dd67)
- Ngắt ngoài của STM32F103 có 16 line ngắt EXTI(n): từ EXTI0 -> EXTI15
  	-  Line0 sẽ chung cho tất cả chân Px0 ở tất cả các Port, với x là tên của Port A, B…
	-  Line0 nếu chúng ta đã chọn chân PA0 (chân 0 ở port A) làm chân ngắt thì tất cả các chân 0 ở các Port khác không được khai báo làm chân ngắt ngoài nữa
	- Line1 nếu chúng ta chọn chân PB1 là chân ngắt thì tất cả chân 1 ở các Port khác không được khai báo làm chân ngắt nữa.
- Các line ngắt sẽ được phân vào các Vector ngắt khác nhau tương ứng.
  ![image](https://github.com/user-attachments/assets/be1c8848-dba8-4b76-8194-71a2a86c80d3)
#### 1.2. Cấu hình ngắt ngoài
- Xác định chân GPIO input, bật clock cho GPIO tương ứng và cho AFIO
  	```c
   	void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	}
- Cấu hình chân input ngắt:
	```c
 	void GPIO_Config(void){
	GPIO_InitTypeDef GPIOInitStruct;
	//Cau hinh cho chan interrupt PA0
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IPU; //Input Pull_up
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIOInitStruct);
	}
- Cấu hình chân ngắt là input và cấu hình thêm có trở kéo lên, kéo xuống tùy vào mục đích sử dụng
- Cấu hình ngắt:
  	```c
  	void EXTI_Config(void) {
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); //PA0
	
	EXTI_InitTypeDef EXTIInitStruct;
	
	EXTIInitStruct.EXTI_Line = EXTI_Line0;//Xac dinh duong ngat (EXTIn) co 18 line va 15 line duoc cau hinh san, 3line mo rong 
	EXTIInitStruct.EXTI_Mode = EXTI_Mode_Interrupt; //Thuc thi ham ngat khi xay ra ngat Even thi khong thuc thi ham ngat
	EXTIInitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//chon canh kich hoat ngat
	EXTIInitStruct.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTIInitStruct);
  	}
- `Hàm GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)` cấu hình chân ở chế độ sử dụng ngắt ngoài
	- GPIO_PortSource: Chọn Port để sử dụng làm nguồn cho ngắt ngoài.
	- GPIO_PinSource: Chọn Pin để cấu hình.
- External Interrupt có struct `Struct EXTI_InitTypeDef` giữ các thuộc tính của ngắt.
	- EXTI_Line: Cấu hình Line ngắt, EXTI_Line0-15
	- EXTI_Mode: Chế độ hoạt động cho ngắt ngoài. Có hai chế độ:
			- EXTI_Mode_Interrupt: Khi ngắt xảy ra sẽ tạo ra ngắt và chạy hàm thực thi.
			- EXTI_Mode_Event: Khi có ngắt chỉ báo cho CPU biết chứ không thực thi hàm phục vụ ngắt.
	- EXTI_Trigger: Xác định khi nào xảy ra Ngắt
		- EXTI_Trigger_Rising: Ngắt cạnh lên
		- EXTI_Trigger_Falling: Ngắt cạnh xuống
		- EXTI_Trigger_Rising_Falling: Cả hai
	- EXTI_LineCmd: Cho phép ngắt ở Line đã cấu hình.
	- EXTI_Init(&EXTIInitStruct): Lưu cài đặt vào thanh ghi.
 #### 1.3. Cấu hình quản lý ngắt NVIC
 - `NVIC (Nested Vectored Interrupt Controller)` chịu trách nhiệm quản lý và xử lý các ngắt. NVIC cho phép MCU xử lý nhiều ngắt từ các nguồn khác nhau, có thể ưu tiên ngắt và hỗ trợ ngắt lồng nhau.
 - Priority Group xác định cách phân chia bit giữa Preemption Priority và SubPriority bằng cách sử dụng hàm `NVIC_PriorityGroupConfig(uint32_t PriorityGroup)`. Trong đó:
	- Preemption Priority xác định mức độ ưu tiên chính của ngắt và quy định ngắt nào có thể được lồng vào.
	- SubPriority: khi các ngắt có cùng mức Preemption Preemption, thì sẽ xem xét tới SubPriority.
 ![image](https://github.com/user-attachments/assets/2484ff88-1b02-44be-87b6-47296b156cd9)
-  Sẽ có 4 bits dùng để quan lý hai loại ưu tiên trên và hàm NVIC_PriorityGroupConfig sẽ chia 4 bits này ra để cấu hình.
- Ví dụ khi cấu hình là NVIC_PriorityGroup_1 Thì một bit sẽ được sử dụng cho Preemption Priority và 3 bits sẽ dùng cho SubPriority.
- Bộ NVIC cấu hình các tham số ngắt và quản lý các vecto ngắt. Các tham số được cấu hình trong NVIC_InitTypeDef, bao gồm:
	- NVIC_IRQChannel: Cấu hình Line ngắt, Enable line ngắt tương ứng với ngắt sử dụng.
   		- Các Line0 đến Line4 sẽ được phân vào các vector ngắt riêng tương ứng EXTI0 -> EXTI4,
		- Line5->Line9 được phân vào vector ngắt EXTI9_5,
		- Line10->Line15 được phân vào vecotr EXTI15_10.
	- NVIC_IRQChannelPreemptionPriority: Cấu hình độ ưu tiên của ngắt.
	- NVIC_IRQChannelSubPriority: Cấu hình độ ưu tiên phụ.
	- NVIC_IRQChannelCmd: Cho phép ngắt.
- Ngoài ra, NVIC_PriorityGroupConfig(); cấu hình các bit dành cho ChannelPreemptionPriority và ChannelSubPriority: 
	- NVIC_PriorityGroup_0: 0 bits for pre-emption priority 4 bits for subpriority
	- NVIC_PriorityGroup_1: 1 bits for pre-emption priority 3 bits for subpriority
 	- NVIC_PriorityGroup_2: 2 bits for pre-emption priority 2 bits for subpriority
	- NVIC_PriorityGroup_3: 3 bits for pre-emption priority 1 bits for subpriority
 	- NVIC_PriorityGroup_4: 4 bits for pre-emption priority 0 bits for subpriority
	```c
 	void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = EXTI0_IRQn; //Dang cau hinh ngat Line0 nen chon vector ngat EXTI0_IRQn
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 1; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 1;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
	}
#### 1.4. Hàm phục vụ ngắt
- Ngắt trên từng line có hàm phục riêng của từng line. Có tên cố định:
	- `EXTIx_IRQHandler()` (x là line ngắt tương ứng) và hàm này đã được định nghĩa sẵn trên hệ thống ở dạng WEAK tức là có thể ghi đè được.
   	```c
   	EXPORT  EXTI0_IRQHandler           [WEAK]  
	EXPORT  EXTI1_IRQHandler           [WEAK]  
	EXPORT  EXTI2_IRQHandler           [WEAK]
	EXPORT  EXTI3_IRQHandler           [WEAK]
	EXPORT  EXTI4_IRQHandler           [WEAK]
	- `Hàm EXTI_GetITStatus(EXTI_Linex)`, Kiểm tra cờ ngắt của line x tương ứng. 
	- `Hàm EXTI_ClearITPendingBit(EXTI_Linex)`: Xóa cờ ngắt ở line x.
- Trong hàm phục vụ ngắt ngoài, chúng ta sẽ thực hiện:
	- Kiểm tra ngắt đến từ line nào, có đúng là line cần thực thi hay không?
	- Thực hiện các lệnh, các hàm.
	- Xóa cờ ngắt ở line.
 - cú pháp hàm:
   	```c
    	void EXTI0_IRQHandler()
	{	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{

	}
	EXTI_ClearITPendingBit(EXTI_Line0);
	}
- ví dụ:
  	```c
   	//Ham xu ly khi co ngat line0
	void EXTI0_IRQHandler(){	
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){ //Kiem tra co ngat cua Line0
		for(int i = 0; i<5; i++){
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			delay_ms(200);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //Xoa co ngat cua line0
	}
	int main(){
		RCC_Config();
		GPIO_Config();
		EXTI_Config();
		NVIC_Config();
		TIMER_config();
		while(1){
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			delay_ms(3000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			delay_ms(3000);
	  }   
	}
### 2. Ngắt Timer
- Tương tự với ngắt ngoài thì ngắt timer cũng cần cấp clock cho AFIO
#### 2.1. Cấu hình ngắt timer
	```c
 	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2; // f = 36Mhz 
	TIM_TimeBaseInitStruct.TIM_Prescaler = 36000-1; //1ms đếm lên một lần
	TIM_TimeBaseInitStruct.TIM_Period =5000-1; //Chu kỳ 5000 tức đếm lên 5000 lần thì reset tức 5s.
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //TIM_IT_Update: Khi Timer cập nhật lại bộ đếm về 0 thì tạo ngắt.
	TIM_Cmd(TIM2, ENABLE);
 #### 2.2. Cấu hình NVIC
 	```c
  	void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cau hinh so bit cua Preemption Priority vaf Sub Prioriry
	
	NVIC_InitTypeDef NVICInitStruct;

	NVICInitStruct.NVIC_IRQChannel = TIM2_IRQn; //Cau hinh channel ngat la Timer2
	NVICInitStruct.NVIC_IRQChannelPreemptionPriority = 0; //mamg gia tri tu 0-3 vi cai dat 2bit cho Preemption Priority
	NVICInitStruct.NVIC_IRQChannelSubPriority = 0;	//mamg gia tri tu 0-3 vi cai dat 2bit cho Sub Priority
	NVICInitStruct.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVICInitStruct);
	}
 - ví dụ nhấp led:
	```c
 	void delay_ms(uint16_t timedelay){
	count = 0;
	while(count < timedelay);
}

void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)){ //Kiem tra co ngat TIM_IT_UPDATe
		count++;
	}
	// Clears the TIM2 interrupt pending bit
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


int main() {
	RCC_Config();
	TIM_Config();
	GPIO_Config();
	NVIC_Config();
	while(1){
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay_ms(2000);
	}
	}
 ### 3. Ngắt truyền thông
- STM32F1 hỗ trọ các ngắt cho các giao thức truyền nhận như SPI, I2C, UART…
- Ở bài này ta sẽ ví dụ với UART ngắt.
- Các ngắt ở SPI, I2C… sẽ được cấu hình tương tự như UART.
#### 3.1. Cấu hình ngắt truyền thông
	```c
	void UART_Config(void){
	USART_InitTypeDef UARTInitStruct;
	UARTInitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //Cau hinh che do: ca truyen va nhan (song cong)
	UARTInitStruct.USART_BaudRate = 9600; //Cau hinh toc do bit
	UARTInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Cau hinh kiem soat luong truyen du lieu tranh viec tran bo dem
	UARTInitStruct.USART_WordLength = USART_WordLength_8b; //Truyen du lieu 8 hoac 9 bit
	UARTInitStruct.USART_Parity = USART_Parity_No;
	UARTInitStruct.USART_StopBits = USART_StopBits_1;
	
	USART_Init(USART1, &UARTInitStruct);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// Khi co du lieu gui toi thi goi ham ngat
	
	USART_Cmd(USART1, ENABLE);
	}
- Hàm `USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)` gồm ba tham số:
	- USART_TypeDef* USARTx: Bộ UART cần cấu hình.
	- uint16_t USART_IT: Chọn nguồn ngắt UART, Có nhiều nguồn ngắt từ UART, ở bài này ta chú ý đến ngắt truyền (USART_IT_TXE) và ngắt nhận (USART_IT_RXNE).
	- FunctionalState NewState: Cho phép ngắt.
 #### 3.2. Cấu hình NVIC 

 void NVIC_Config(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

}
#### 3.3. Hàm phục vụ ngắt
- Hàm USARTx_IRQHandler() sẽ được gọi nếu xảy ra ngắt trên Line ngắt UART đã cấu hình.
- Hàm USART_GetITStatus kiểm tra các cờ ngắt UART. Hàm này nhận 2 tham số là bộ USART và cờ tương ứng cần kiểm tra:
	- USART_IT_RXNE: Cờ ngắt nhận, cờ này set lên 1 khi bộ USART phát hiện data truyền tới.
	- USART_IT_TXE: Cờ ngắt truyền, cờ này set lên 1 khi USART truyền data xong.
	- Có thể xóa cờ ngắt, gọi hàm USART_ClearITPendingBit để đảm bảo không còn ngắt trên line (thông thường cờ ngắt sẽ tự động xóa).
 - ví dụ:
	```c
 	
	uint8_t UART_ReceiveChar(USART_TypeDef *USARTx){
		uint8_t  data = 0x00;
		while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)==RESET); //cho co nhan bat len 1 ( nhan hoan tat) thi tra ve gia tri data
		data = USART_ReceiveData(USARTx);
		return data;
	}
	
	uint8_t Data_Receive;
	
	void USART1_IRQHandler(){
		if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET){
			Data_Receive = UART_ReceiveChar(USART1);
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	
	uint16_t count = 0;
	int main() {
		RCC_Config();
		GPIO_Config();
		UART_Config();
		TIMER_config();
		NVIC_Config();
		while(1){
			count++;
			delay_ms(1000);
		}
	}
 - Tại hàm phục vụ ngắt sẽ thực thi khi có dữ liệu được nhận tại USART1 để không bị mất dữ liệu và tại chương trình chính sẽ cộng biến Count lên 1.
</details>
<details><summary>LESSON 9: ADC</summary>
    <p>
        
## LESSON 9: ADC
### 1. Định nghĩa
- ADC (Analog-to-Digital Converter) là 1 mạch điện tử lấy điện áp tương tự làm tín hiệu đầu vào và chuyển đổi nó thành dữ liệu số (1 giá trị đại diện cho mức điện áp trong mã nhị phân).
![image](https://github.com/user-attachments/assets/b127b2ba-814e-4e91-808d-a4bdb3488e5f)
- ADC hoạt động theo cách chia mức tín hiệu tương tự thành nhiều mức khác nhau.
![image](https://github.com/user-attachments/assets/97f66bb1-1dd2-4238-8944-0059d4c9a84f)
- Bằng việc so sánh giá trị điện áp mỗi lần lấy mẫu với 1 mức nhất định, ADC chuyển đổi tín hiệu tương tự và mã hóa các giá trị về giá trị nhị phân tương ứng.
- Khả năng chuyển đổi của ADC được quyết định bởi 2 yếu tố chính:
	- Độ phân giải: Số bit mà ADC sử dụng để mã hóa tín hiệu. Có thể xem như là số mức mà tín hiệu tương tự được biểu diễn. ADC có độ phân giải càng cao thì cho ra kết quả chuyển đổi càng chi tiết.

	- Tần số/chu kì lấy mẫu: tốc độ/khoảng thời gian giữa 2 lần mã hóa. Tần số lấy mẫu càng lớn thì tín hiệu sau khi chuyển đổi sẽ có độ chính xác càng cao. Khả năng tái tạo lại tín hiệu càng chính xác.
	  - `Tần số lấy mẫu = 1/(thời gian lấy mẫu+ Tg chuyển đổi.)`

-> **Tần số lấy mẫu phải lớn hơn tần số của tín hiệu ít nhất 2 lần để đảm bảo độ chính xác khi khôi phục lại tín hiệu.**
### 2. ADC trong STM32
- STM32F103C8 có 2 bộ ADC đó là ADC1 và ADC2 với nhiều mode hoạt động.
- Kết quả chuyển đổi được lưu trữ trong thanh ghi 16 bit.

	- Độ phân giải 12 bit.
	- Có các ngắt hỗ trợ.
	- Có thể điều khiển hoạt động ADC bằng xung Trigger.
	- Thời gian chuyển đổi nhanh : 1us tại tần số 65Mhz.
	- Có bộ DMA giúp tăng tốc độ xử lí.
- Các chế độ của ADC: Có 16 kênh ADC nhưng tại một thời điểm chỉ có một kênh được chuyển đổi dữ liệu
- Regular Conversion:
	- Single: ADC chỉ đọc 1 kênh duy nhất, và chỉ đọc khi nào được yêu cầu.
	- Single Continuous: ADC sẽ đọc một kênh duy nhất, nhưng đọc dữ liệu nhiều lần liên tiếp (Có thể được biết đến như sử dụng DMA để đọc dữ liệu và ghi vào bộ nhớ).
	- Scan: Multi-Channels: Quét qua và đọc dữ liệu nhiều kênh, nhưng chỉ đọc khi nào được yêu cầu.
	- Scan: Continuous Multi-Channels Repeat: Quét qua và đọc dữ liệu nhiều kênh, nhưng đọc liên tiếp nhiều lần giống như Single Continous.
- Injected Conversion:
- Trong trường hợp nhiều kênh hoạt động. Khi kênh có mức độ ưu tiên cao hơn có thể tạo ra một Injected Trigger. Khi gặp Injected Trigger thì ngay lập tức kênh đang hoạt động bị ngưng lại để kênh được ưu tiên kia có thể hoạt động.
-  Giả sử ta cần đo điện áp tối thiểu là 0V và tối đa là 3.3V, trong STM32 sẽ chia 0 → 3.3V thành 4096 khoảng giá trị (từ 0 → 4095, do 2^12 = 4096), giá trị đo được từ chân IO tương ứng với 0V sẽ là 0, tương ứng với 1.65V là 2047 và tương ứng 3.3V sẽ là 4095.
#### 2.1. Cấp clock cho ADC
- Cấp xung cho bộ ADC và GPIO để cử dụng Pin, đồng thời cũng phải cấp cho bộ AFIO.
	```c
 	void RCC_Config(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
#### 2.2. Cấu hình GPIO 
-  Cấu hình chân PA0 với chức năng Analog Input GPIO_Mode_AIN
	```c
 	void GPIO_Config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; //Analog input
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
 #### 2.3. Cấu hình ADC


 void ADC_Config(void){
	ADC_InitTypeDef ADC_InitStruct;
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent; //Hoat dong nhu ADC binh thuong, doc lap voi nhau va khong can kich hoat
	ADC_InitStruct.ADC_NbrOfChannel = 1; //NumberOfChannel so luong kenh can cau hinh (1-16)
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//Hoat dong o che do Continous hay khong
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //Cau hinh cho phep su dung Trigger (tin hieu de bat dau chuyen doi ADC)
	ADC_InitStruct.ADC_ScanConvMode = DISABLE; //Co su dung Scan de quet nhieu kenh khong
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//Cau hinh de can le cho Data (ADC 12bit luu vao 16bit bi du 4 bit), ghi vao LSB hay MSB

	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	//Bat dau qua trinh chuyen doi (Vi chon che do Continous nen chi can goi 1 lan)
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

- ADC_Mode: Cấu hình chế độ hoạt động cho bộ ADC
	- ADC_Mode_Independent: Các kênh hoạt động độc lập với nhau.                 
	- ADC_Mode_RegInjecSimult: Sử dụng cả hai mode Regular và Injected.                      
	- ADC_Mode_InjecSimult: Hoạt động chế độ Injected                  
	- ADC_Mode_RegSimult: Hoạt động ở chế độ Regular                                      
- ADC_NbrOfChannel: Số lượng ADC cần cấu hình 1-16

- ADC_ContinuousConvMode: Có chuyển đổi ADC liên tục hay không ENABLE hoặc DISABLE

- ADC_ExternalTrigConv: Cấu hình sử dụng kích chuyển đổi ADC ở bên ngoài như Timer, External Trigger.

- ADC_ScanConvMode: Có sử dụng Mode Scan để quét qua nhiều kênh hay không
- ADC_DataAlign: Cấu hình căn lề cho data. Vì bộ ADC xuất ra giá trị 12bit, được lưu vào biến 16 hoặc 32 bit nên phải căn lề các bit về trái hoặc phải. Nếu căn phải thì dữ liệu đọc giữ nguyên, căn trái dữ liệu đọc gấp 16 lần thực tế nếu được lưu vào biến 16 bits

- Ngoài các tham số trên, cần cấu hình thêm thời gian lấy mẫu, thứ tự kênh ADC khi quét, - - 
- ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)

	- Rank: Ưu tiên của kênh ADC. Cao nhất 1, thấp nhất 16
	- SampleTime: Thời gian lấy mẫu tín hiệu.
   		```c
		 ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
		 ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
		 ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
		 ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
		 ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
		 ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
		 ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
- ADC_SoftwareStartConvCmd(ADCx, NewState): Bắt đầu quá trình chuyển đổi
- ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState): Bắt đầu quá trình chuyển đổi.
 - ADC_GetConversionValue(ADC_TypeDef* ADCx): Đọc giá trị chuyển đổi được ở các kênh ADC tuần tự.
- ADC_GetDualModeConversionValue(void): Trả về giá trị chuyển đổi cuối cùng của ADC1, ADC2 ở chế độ kép. Trả về thanh ghi 32 bits, 16 bits đầu của ADC2 và 16 bits sau của ADC1
### 3. Bộ lọc Kalman
Bộ lọc Kalman, được Rudolf (Rudy) E. Kálmán công bố năm 1960, là thuật toán sử dụng chuỗi các giá trị đo lường, bị ảnh hưởng bởi nhiễu hoặc sai số để ước đoán biến số nhằm tăng độ chính xác. Kalman
![image](https://github.com/user-attachments/assets/861bf878-7021-402d-99b9-f8b2eab37023)

Gọi hàm SimpleKalmanFilter(float mea_e, float est_e, float q)); Để khởi các giá trị sai số ước tính, sai số đo lường và sai số quá trình ban đầu. - mea_e: Sai số đo lường của phần cứng - est_e: Sai số dự đoán. - q: Sai số nhiễu quá trình từ tính hiệu đến bộ chuyển đổi ADC.
</details>
<details><summary>LESSON 10: DMA</summary>
    <p>
        
## LESSON 10: DMA
### 1. Khái niệm
DMA (Direct Memory Access) là một cơ chế trong hệ thống nhúng và máy tính cho phép các thiết bị ngoại vi (ví dụ: ADC, SPI, UART, v.v.) hoặc bộ nhớ trong hệ thống (như RAM, Flash) truyền hoặc nhận dữ liệu trực tiếp với bộ nhớ mà không cần sự can thiệp trực tiếp của CPU.

Trong các hệ thống không sử dụng DMA, khi thiết bị ngoại vi cần truyền hoặc nhận dữ liệu (ví dụ, cảm biến gửi dữ liệu qua SPI hoặc UART), CPU phải thực hiện từng thao tác đọc hoặc ghi dữ liệu từng byte từ bộ nhớ sang thanh ghi của thiết bị ngoại vi hoặc ngược lại. Quá trình này tốn nhiều thời gian CPU và làm giảm hiệu suất xử lý chung của hệ thống, đặc biệt khi khối lượng dữ liệu lớn hoặc tốc độ truyền nhanh.
![image](https://github.com/user-attachments/assets/e4925039-6248-4875-a8c0-a84880cc6fe3)

DMA giải quyết vấn đề này bằng cách cho phép CPU chỉ thiết lập cấu hình ban đầu thông qua Handshake cho bộ điều khiển DMA (DMA controller), bao gồm địa chỉ bộ nhớ nguồn, địa chỉ bộ nhớ đích, kích thước dữ liệu cần truyền, và các chế độ hoạt động. Sau đó, bộ điều khiển DMA sẽ tự động quản lý việc truyền dữ liệu trực tiếp giữa bộ nhớ và thiết bị ngoại vi mà không cần CPU phải can thiệp vào từng bước.

Khi hoàn tất quá trình truyền, DMA gửi tín hiệu ngắt (interrupt) để thông báo cho CPU, từ đó CPU có thể xử lý các tác vụ tiếp theo. Nhờ vậy, CPU được giải phóng để thực hiện các nhiệm vụ khác, nâng cao hiệu suất tổng thể và giảm độ trễ của hệ thống.
![image](https://github.com/user-attachments/assets/a60a6f7f-53e0-4cb0-9065-aab4f6ccb635)
- CPU (1) cấu hình và kích hoạt DMA (4) hoạt động.
- Ngoại vi (5) sẽ sử dụng DMA Request (6) để yêu cầu DMA (4) gửi/nhận dữ liệu ngoại vi.
- DMA (4) tiến hành thực hiện yêu cầu từ DMA Request (6).
- Lấy dữ liệu từ SRAM (2) thông qua Bus Matrix (3) <-> Đi qua các đường bus ngoại vi <-> Truy cập các thanh vi của ngoại vi (5).
- Khi kết thúc, DMA (4) kích hoạt ngắt báo cho CPU (1) biết là quá trình hoan tất.
### 2. DMA trong STM32
Trên vi điều khiển STM32, DMA hoạt động với các thành phần chính sau:
- DMA Stream: Là các luồng DMA, mỗi luồng có thể truyền một loại dữ liệu cụ thể (ví dụ: truyền từ bộ nhớ đến một thiết bị ngoại vi hoặc ngược lại).
- DMA Channel: Một kênh DMA cụ thể sẽ liên kết với một thiết bị ngoại vi (ví dụ: SPI, USART).
- DMA Controller (DMA1, DMA2): Điều khiển tất cả các luồng DMA.
- DMA Interrupt: DMA có thể tạo interrupt (ngắt) khi hoàn thành một công việc, giúp CPU biết khi nào dữ liệu đã được truyền xong.

Ví dụ: STM32F4 có 2 DMA controllers (DMA1 và DMA2), mỗi controller có 8 streams, và mỗi stream hỗ trợ nhiều channel. Việc kết hợp stream và channel giúp chọn đúng thiết bị ngoại vi và kiểu truyền.

STM32F103C8T6 có hai bộ DMA. DMA1 gồm 7 kênh, DMA2 gồm 5 kênh.
![image](https://github.com/user-attachments/assets/e458feea-5df8-45af-a7bb-efa96f9122bf)

- Hai chế độ hoạt động chính:
	- Normal mode (Chế độ bình thường): Truyền dữ liệu một lần theo số lượng byte đã cấu hình, sau đó dừng.
	- Circular mode (Chế độ vòng tròn): Truyền dữ liệu liên tục, tự động quay lại đầu khi kết thúc, phù hợp cho các ứng dụng thu thập dữ liệu liên tục như ADC.
-  Cấu hình kênh linh hoạt:
	- Mỗi kênh DMA có thể được cấu hình riêng biệt về địa chỉ nguồn, địa chỉ đích, kích thước dữ liệu, hướng truyền, và chế độ hoạt động. Điều này cho phép mỗi kênh phục vụ cho nhu cầu truyền dữ liệu khác nhau trong hệ thống.
- Chia sẻ kênh cho nhiều ngoại vi:
	- Một kênh DMA có thể được ánh xạ để phục vụ cho nhiều ngoại vi khác nhau. Tuy nhiên, mỗi kênh chỉ hoạt động cho một ngoại vi tại một thời điểm, không thể đồng thời phục vụ nhiều ngoại vi.
- Ưu tiên kênh (Priority Levels):
	- Các kênh DMA có mức ưu tiên khác nhau (thấp đến rất cao). Khi nhiều kênh cùng yêu cầu truy cập bus DMA, kênh có ưu tiên cao hơn sẽ được phục vụ trước. Điều này giúp đảm bảo các tác vụ quan trọng hơn được xử lý kịp thời.
- Hệ thống ngắt (Interrupt) với nhiều cờ báo:
	- DMA trên STM32 hỗ trợ 5 loại cờ ngắt chính để theo dõi và xử lý quá trình truyền dữ liệu:

	- DMA Half Transfer (HT): Báo khi quá trình truyền đã hoàn thành một nửa lượng dữ liệu.

	- DMA Transfer Complete (TC): Báo khi truyền dữ liệu hoàn tất.

	- DMA Transfer Error (TE): Báo lỗi trong quá trình truyền.

	- DMA FIFO Error (FE): Báo lỗi liên quan đến FIFO (nếu DMA sử dụng FIFO).

	- Direct Mode Error (DME): Báo lỗi khi sử dụng chế độ truyền trực tiếp không qua FIFO.

Nhờ các cờ báo này, phần mềm có thể linh hoạt xử lý các sự kiện truyền dữ liệu, theo dõi tiến trình, và xử lý lỗi nhanh chóng, nâng cao độ tin cậy của hệ thống.
### 3. Cấu hình DMA trên STM32
#### 3.1. Cấp xung clock thông qua bus AHB
	```c
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_SPI1| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 #### 3.2. Cấu hình DMA
- Các tham số cho bộ DMA được cấu hình trong struct DMA_InitTypeDef. Gồm:
	- `DMA_PeripheralBaseAddr`: Cấu hình địa chỉ của ngoại vi cho DMA. Đây là địa chỉ mà DMA sẽ lấy data hoặc truyền data tới cho ngoại vi. ( ô địa chỉ đầu tiên)
	- `DMA MemoryBaseAddr`: Cấu hình địa chỉ vùng nhớ cần ghi/ đọc data .
	- `DMA_DIR`: Cấu hình hướng truyền DMA, từ ngoại vi tới vùng nhớ PeripheralSRC hay từ vùng nhớ tới ngoại vi PeripheralDST.
	- `DMA_BufferSize`:Kích thước của buffer, số lượng phần tử dữ liệu mà DMA sẽ truyền hoặc nhận. Đây là số lượng đơn vị dữ liệu (không phải số byte mà phụ thuộc vào kích thước data size) mà DMA xử lý trong một lần truyền. Ví dụ: Nếu bạn truyền 100 phần tử mỗi phần tử là 1 byte, DMA_BufferSize là 100.
	- `DMA_PeripheralInc`: Cấu hình địa chỉ ngoại vi có tăng sau khi truyền DMA hay không. Nếu Enable, địa chỉ ngoại vi sẽ tăng lên theo kích thước data (byte, halfword, word) sau mỗi lần truyền, dùng cho các ngoại vi có vùng địa chỉ liên tục. Nếu Disable, địa chỉ ngoại vi giữ nguyên, thường dùng cho thanh ghi dữ liệu đơn của ngoại vi (ví dụ UART DR).
	- `DMA Memory Inc`: Cấu hình địa chỉ bộ nhớ có tăng lên sau khi truyền DMA hay không. Enable: địa chỉ bộ nhớ tăng lên theo kích thước data để lưu dữ liệu liên tục trong vùng nhớ. Disable: địa chỉ bộ nhớ giữ nguyên, dùng khi truyền dữ liệu lặp lại đến hoặc từ cùng một vị trí bộ nhớ.
	- `DMA_PeripheralDataSize`: Cấu hình độ lớn data của ngoại vi. Có thể là 8-bit (byte), 16-bit (halfword) hoặc 32-bit (word) tùy loại ngoại vi và ứng dụng. DMA sẽ đọc/ghi theo kích thước này.
	- `DMA_MemoryDataSize`: Cấu hình độ lớn data của bộ nhớ, Byte, halfWord hoặc Word
	- `DMA_Mode`: Cấu hình mode hoạt động là Normal hay Circular
	- `DMA_Priority`: Cấu hình độ ưu tiên cho kênh DMA.
	- `DMA_M2M`: Kích hoạt sử dụng truyền từ bộ nhớ đếm bộ nhớ cho kênh DMA Enable hoặc Disable

- Khởi tạo DMA với DMA_Init(): Cấu hình các tham số cho kênh DMA (ở đây là DMA1_Channel2) dựa trên cấu trúc DMA_InitStruct mà bạn đã thiết lập (gồm các tham số như địa chỉ ngoại vi, bộ nhớ, kích thước buffer, hướng truyền, v.v).  Thiết lập xong cấu hình này, DMA đã biết cách hoạt động cho kênh này.

	```c
 
	DMA_Init(DMA1_Channel2, &DMA_InitStruct);
 

-  Kích hoạt kênh DMA với DMA_Cmd():  Bật kênh DMA, cho phép nó bắt đầu hoạt động.
	```c
	DMA_Cmd(DMA1_Channel2, ENABLE);

- Kích hoạt DMA cho ngoại vi cụ thể với hàm riêng biệt, ví dụ SPI: Cấu hình cho phép SPI1 sử dụng DMA để nhận dữ liệu (DMA request cho Rx).
	```c
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

- Ý nghĩa: DMA1_Channel2 trong ví dụ này được dùng cho SPI1_RX, tức là kênh 2 của DMA1 kết nối với chức năng nhận dữ liệu của SPI1., ngoại vi SPI sẽ kích hoạt tín hiệu yêu cầu DMA khi có dữ liệu cần truyền hoặc nhận, tạo sự phối hợp giữa DMA và SPI.
- Sau khi DMA được kích hoạt và ngoại vi yêu cầu DMA, bộ DMA tự động xử lý việc lấy dữ liệu từ SPI và ghi vào bộ nhớ (hoặc ngược lại nếu truyền). Bạn không cần phải can thiệp thêm trong code ngoại trừ xử lý interrupt (nếu có).
### 4. PWM
- Tín hiệu PWM dùng để điều khiển động cơ servo gồm hai yếu tố chính:
	- Tần số: thường là 50 Hz, tức là tín hiệu lặp lại mỗi 20 ms.
	- Độ rộng xung: là thời gian tín hiệu ở mức cao trong mỗi chu kỳ, thường từ 1 ms đến 2 ms, tương ứng với góc quay của servo từ 0° đến 180°.
- Chu kỳ nhiệm vụ (duty cycle) là tỉ lệ độ rộng xung trên chu kỳ, dao động khoảng từ 5% đến 10%. Độ rộng xung càng lớn thì servo quay càng xa góc tối đa.
![image](https://github.com/user-attachments/assets/a06ea4c5-a05e-411c-a0e4-09b780e1958f)
- Công thức tính toán độ rộng xung cho Servo:
`pulseWidth = MIN_PULSE_WIDTH + (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle / 180`
	 - MIN_PULSE_WIDTH là độ rộng xung cho góc 0 độ (thường là 1000µs).
	- MAX_PULSE_WIDTH là độ rộng xung cho góc 180 độ (thường là 2000µs).
	- angle là góc mà servo xoay đến.
### 4.1. Cấu hình PWM trong STM32
- Ta phải cấu hình chân chế độ AF_PP để gán chân GPIO với 1 kênh của timer mà ta cấu hình chế độ PWM.
	```c
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // PA0 là TIM2_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Chế độ Alternate Function Push-Pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
- Cấu hình timer chế độ PWM trong struct TIM_OCInitTypeDef:

	- TIM_OCMode: Chọn chế độ hoạt động cho Output Compare. PWM1 thì kênh timer sẽ ở mức 1 khi nhỏ hơn TIM_Pulse và mức 0 khi lớn hơn. PWM2 ngược lại so với PWM1.
	- TIM_OutputState: cho phép tín hiệu PWM được xuất ra từ chân tương ứng của MCU.
	- TIM_Pulse: Đặt giá trị đầu cho độ rộng xung (giá trị timer dùng để so sánh).
	- TIM_OCPolarity: Đặt cực tính của tín hiệu ngõ ra TIM_OCPolarity_High sẽ làm xung ở mức 1 (HIGH) khi giá trị đếm còn nhỏ hơn TIM_Pulse còn TIM_OCPolarity_Low sẽ làm xung ở mức 0 (LOW) khi giá trị đếm còn hơn hơn TIM_Pulse. Gọi hàm TIM_OCxInit(); để cấu hình cho kênh x tương ứng.
- Hàm TIM_OCxPreloadConfig(); cấu hình Timer hoạt động với tính năng Preload (TIM_OCPreload_Enable) hay không (TIM_OCPreload_Disable).
	- Tính năng Preload là tính năng mà hệ thống sẽ chờ cho tới khi timer tạo ra sự kiện Update Event thì mới nạp lại giá trị so sánh mới (TIM_Pulse)
	- Sự kiện Update Event là sự kiện xảy ra khi timer đã đếm đến giá trị tối đa được cấu hình và sẽ quay lại 0 để bắt đầu chuu kỳ mới. Gọi hàm TIM_Cmd(); để cho phép Timer hoạt động.
 - 	```c
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_Cmd(TIM2);
    
Để thay đổi độ rộng xung xuất ra, sử dụng hàm TIM_SetComparex(TIMx, pulseWidth); với Timer sử dụng là TIMx và độ rộng pulseWidth.

