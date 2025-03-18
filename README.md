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


	





