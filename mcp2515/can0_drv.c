/*
 * CAN bus driver for Microchip 2515 CAN Controller with GPIO simulate SPI
 *
 * MCP2515 support and bug fixes by horotororensu
 * <han.liu@keluofeite.com>
 *
 * Copyright 2017 keluofeite.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <mach/w55fa92_reg.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include "mcp2515.h"

/* MCP2515 Pin definition */
#define MCP2515_SCK0	(writel(readl(REG_GPIOD_DOUT) & ~((1 << 12)), REG_GPIOD_DOUT))
#define MCP2515_SCK1	(writel(readl(REG_GPIOD_DOUT) | ((1 << 12)), REG_GPIOD_DOUT))
#define MCP2515_MISO	((readl(REG_GPIOD_PIN) & (1 << 14)) > 0 ? 1 : 0)
#define MCP2515_MOSI0   (writel(readl(REG_GPIOD_DOUT) & ~((1 << 15)), REG_GPIOD_DOUT))	
#define MCP2515_MOSI1	(writel(readl(REG_GPIOD_DOUT) | ((1 << 15)), REG_GPIOD_DOUT))
#define MCP2515_CS0  	(writel(readl(REG_GPIOD_DOUT) & ~((1 << 13)), REG_GPIOD_DOUT))	
#define MCP2515_CS1 	(writel(readl(REG_GPIOD_DOUT) | ((1 << 13)), REG_GPIOD_DOUT))

/* MCP2515 baud rate */
#define	CAN_10Kbps	0x31
#define CAN_25Kbps	0x13
#define CAN_50Kbps	0x09
#define CAN_100Kbps	0x04
#define CAN_125Kbps	0x03
#define CAN_250Kbps	0x01
#define	CAN_500Kbps	0x00

/* MCP2515 ioctl commands */
#define SETBAUD		0
#define SETMASK		1
#define SETID		3
#define STATE		4
#define GETID		5

/* set value*/
static int baudrate;

/* 
 * function: SPI_ReadByte
 * description: Read 1Byte data by SPI
 * arguments: nothing
 * return: 
 * 		rByte: 1Bytes data read by SPI
 */
static unsigned char SPI_ReadByte(void)
{
	unsigned char i, rByte = 0;
	MCP2515_SCK0;
	for(i=0; i<8; i++)
	{
		MCP2515_SCK1;
		rByte <<= 1;
		rByte |= MCP2515_MISO;
		MCP2515_SCK0;	
	}
	return rByte;
}

/* 
 * function: SPI_SendByte
 * description: SPI send 1Byte data
 * arguments: 
 * 		dt: send data
 * return: nothing
 */
static void SPI_SendByte(unsigned char dt)
{
	unsigned char i;
	for(i=0; i<8; i++)
	{	
		MCP2515_SCK0;
		if((dt << i) & 0x80)
			MCP2515_MOSI1;
		else
			MCP2515_MOSI0;					
		MCP2515_SCK1;
	}
	MCP2515_SCK0;
}

/* 
 * function: can0_writeByte
 * description: Write 1Bytes of data to the address register of MCP2515 
 *  		    through SPI
 * arguments: 
 * 		addr: MCP2515 register address
 * 		dat: Data to be written
 * return: nothing
 */
static void can0_writeByte(unsigned char addr,unsigned char dat)
{
	MCP2515_CS0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_WRITE);	//发送写命令
	SPI_SendByte(addr);			//发送地址
	SPI_SendByte(dat);			//写入数据
	MCP2515_CS1;				//置MCP2515的CS为高电平 

}

/* 
 * function: can0_readByte
 * description: Read 1Bytes of data from the address register of MCP2515 
 *  		    through SPI
 * arguments: 
 * 		addr: MCP2515 register address
 * return: 
 * 		rByte: 1Bytes data read
 */
static unsigned char can0_readByte(unsigned char addr)
{
	unsigned char rByte;
	MCP2515_CS0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_READ);		//发送读命令
	SPI_SendByte(addr);			//发送地址
	rByte = SPI_ReadByte();		//读取数据
	MCP2515_CS1;				//置MCP2515的CS为高电平
	return rByte;				//返回读到的一个字节数据
}

/* 
 * function: MCP2515_Reset
 * description: Reset MCP2515 and set as configuration mode
 * arguments: nothing
 * return: nothing
 */
static void MCP2515_Reset(void)
{
	MCP2515_CS0;				//置MCP2515的CS为低电平
	SPI_SendByte(CAN_RESET);	//发送寄存器复位命令
	MCP2515_CS1;				//置MCP2515的CS为高电平
}

/* 
 * function: can0_init
 * description: Initialize MCP2515
 * arguments: 
 * 		rate: The baud rate of MCP2515
 * return: nothing
 */
static void can0_init(int rate)
{
	unsigned char temp = 0, cnf1;
	MCP2515_Reset();	//发送复位指令软件复位MCP2515

	//配置引脚PD12 PD13 PD15
	writel(readl(REG_GPIOD_OMD) | (1 << 12) | (1 << 13) | (1 << 15), REG_GPIOD_OMD);  // output
	writel(readl(REG_GPIOD_PUEN) | ((1 << 12) | (1 << 13) | (1 << 15)), REG_GPIOD_PUEN); // pull up
	writel(readl(REG_GPIOD_DOUT) & ~((1 << 12) | (1 << 13) | (1 << 15)), REG_GPIOD_DOUT); // low

	//配置引脚PD14
	writel(readl(REG_GPIOD_OMD) & ~((1 << 14)), REG_GPIOD_OMD); // input
	writel(readl(REG_GPIOD_PUEN) | ((1 << 14)), REG_GPIOD_PUEN); // pull-up

	//设置波特率为125Kbps
		
	//set CNF1,SJW=00,长度为1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
	printk("can0 set up\n");
	can0_writeByte(CNF1,rate);
	//set CNF2,SAM=0,在采样点对总线进行一次采样，PHSEG1=(2+1)TQ=3TQ,PRSEG=(0+1)TQ=1TQ
	can0_writeByte(CNF2,0x80|PHSEG1_3TQ|PRSEG_1TQ);
	//set CNF3,PHSEG2=(2+1)TQ=3TQ,同时当CANCTRL.CLKEN=1时设定CLKOUT引脚为时间输出使能位
	can0_writeByte(CNF3,PHSEG2_3TQ);

	can0_writeByte(TXB0SIDH,0xFF);//发送缓冲器0标准标识符高位
	can0_writeByte(TXB0SIDL,0xE0);//发送缓冲器0标准标识符低位
	can0_writeByte(RXB0SIDH,0x00);//清空接收缓冲器0的标准标识符高位
	can0_writeByte(RXB0SIDL,0x00);//清空接收缓冲器0的标准标识符低位
	can0_writeByte(RXB0CTRL,0x20);//仅仅接收标准标识符的有效信息
	can0_writeByte(RXB0DLC,DLC_8);//设置接收数据的长度为8个字节

	can0_writeByte(RXF0SIDH,0xFF);//配置验收滤波寄存器n标准标识符高位
	can0_writeByte(RXF0SIDL,0xE0);//配置验收滤波寄存器n标准标识符低位
	can0_writeByte(RXM0SIDH,0xFF);//配置验收屏蔽寄存器n标准标识符高位
	can0_writeByte(RXM0SIDL,0xE0);//配置验收屏蔽寄存器n标准标识符低位

	can0_writeByte(CANINTF,0x00);//清空CAN中断标志寄存器的所有位(必须由MCU清空)
	can0_writeByte(CANINTE,0x01);//配置CAN中断使能寄存器的接收缓冲器0满中断使能,其它位禁止中断

	can0_writeByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//将MCP2515设置为正常模式,退出配置模式
	temp=can0_readByte(CANSTAT);//读取CAN状态寄存器的值
	if(OPMODE_NORMAL!=(temp&&0xE0))//判断MCP2515是否已经进入正常模式
	{
		can0_writeByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//再次将MCP2515设置为正常模式,退出配置模式
	} else {
		printk("can0 mode: normal mode\n");
	}

	cnf1 = can0_readByte(CNF1);
	if (cnf1 == 0x00) {
		printk("can0 Baud rate: 500Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x01) {
		printk("can0 Baud rate: 250Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x03) {
		printk("can0 Baud rate: 125Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x04) {
		printk("can0 Baud rate: 100Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x09) {
		printk("can0 Baud rate: 50Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x13) {
		printk("can0 Baud rate: 25Kbps\n");
		baudrate = CAN_500Kbps;
	}
	if (cnf1 == 0x31) {
		printk("can0 Baud rate: 10Kbps\n");
		baudrate = CAN_500Kbps;
	}
}

/*
 * function: CAN_Send_Buffer
 * description: Send data through CAN
 * arguments:
 * 		CAN_TX_Buf: data to be sent
 * 		len: length of data
 * return: nothing
 */
static void CAN_Send_Buffer(unsigned char *CAN_TX_Buf, unsigned char len)
{
	unsigned char j, dly, count;

	count = 0;
	while(count < len)
	{
		dly = 0;
		while((can0_readByte(TXB0CTRL) & 0x08) && (dly < 50))//快速读某些状态指令,等待TXREQ标志清零
		{
			dly++;
		}
															
		for(j = 0;j < 8; )
		{
			can0_writeByte(TXB0D0 + j,CAN_TX_Buf[count++]);//将待发送的数据写入发送缓冲寄存器
			j++;
			if(count>=len) break;
		}

		can0_writeByte(TXB0DLC, j);//将本帧待发送的数据长度写入发送缓冲器0的发送长度寄存器
		MCP2515_CS0;
		can0_writeByte(TXB0CTRL, 0x08);//请求发送报文
		MCP2515_CS1;
	}
}

/*
 * function: CAN_Receive_Buffer
 * description: Recieve data from CAN
 * arguments: 
 * 		CAN_RX_Buf: pointer of data buffer to be received
 * return:
 * 		len: length of data received
 */
static unsigned char CAN_Receive_Buffer(unsigned char *CAN_RX_Buf)
{
	unsigned char i = 0, len = 0, temp = 0;

	temp = can0_readByte(CANINTF);

	if(temp & 0x01)
	{
		len = can0_readByte(RXB0DLC);//读取接收缓冲器0接收到的数据长度(0~8个字节)
		while(i < len)
		{	
			CAN_RX_Buf[i] = can0_readByte(RXB0D0 + i);//把CAN接收到的数据放入指定缓冲区
			i++;
		}
		can0_writeByte(CANINTF,0);//清除中断标志位(中断标志寄存器必须由MCU清零)
	}
	return len;
}

/* ======================================================================================= */

static int can0_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t can0_read(struct file *fp, char __user *buffer, size_t count, loff_t *off)
{
	struct mcp2515_st *m;
	struct mcp_user_st mu;
	int ret;

	m = fp->private_data;
	ret = CAN_Receive_Buffer(mu.data);
	mu.can_dlc = ret;

	ret = copy_to_user(buffer, &mu, sizeof(mu));
	if (ret) {
		ret = -EFAULT;
		goto copy_error;
	}
	
	return mu.can_dlc;
copy_error:
	return ret;
}

static ssize_t can0_write(struct file *fp, const char __user *buffer, size_t count, loff_t *off)
{
	struct mcp_user_st mu;
    struct mcp2515_st *m;
	int ret;

	m = fp->private_data;
	if (count != sizeof(mu)) {
		return -EINVAL;
	}
	ret = copy_from_user(&mu, buffer, count);
	if (ret) {
		ret = -EFAULT;
		goto copy_error;
	}

	if (count > 8)
		CAN_Send_Buffer(mu.data, 8);
	else 
		CAN_Send_Buffer(mu.data, count);

	return count;
copy_error:
	return ret;
}

int can0_close(struct inode *no, struct file *fp)
{
	return 0;
}

long can0_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct mcp2515_st *m;
	int txidh, txidl;
	int maskh, maskl;
	int h, l;
	int temp;

	m = fp->private_data;

	switch (cmd) {
		//set baud rate 
		case SETBAUD:
			can0_init(arg);
			break;
		//set mask
		case SETMASK:
			MCP2515_Reset();	//发送复位指令软件复位MCP2515
			//set CNF1,SJW=00,长度为1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
			printk("Set CAN0 mask\n");
			can0_writeByte(CNF1, baudrate);
			//set CNF2,SAM=0,在采样点对总线进行一次采样，PHSEG1=(2+1)TQ=3TQ,PRSEG=(0+1)TQ=1TQ
			can0_writeByte(CNF2,0x80|PHSEG1_3TQ|PRSEG_1TQ);
			//set CNF3,PHSEG2=(2+1)TQ=3TQ,同时当CANCTRL.CLKEN=1时设定CLKOUT引脚为时间输出使能位
			can0_writeByte(CNF3,PHSEG2_3TQ);

			if (arg == 0) {
				//取消屏蔽
				printk("set CAN0 mask 0x0\n");
				can0_writeByte(RXM0SIDH, 0x00);//配置验收屏蔽寄存器n标准标识符高位
				can0_writeByte(RXM0SIDL, 0x00);//配置验收屏蔽寄存器n标准标识符低位
				h = can0_readByte(RXM0SIDH);
				l = can0_readByte(RXM0SIDL);
				printk("h = %x, l = %x\n", h, l);
				if (h == 0 && l == 0)
					printk("Set CAN0 mask ID: 0x%x\n", 0);
			} else {
				can0_writeByte(RXM0SIDH, 0xFF);//配置验收屏蔽寄存器n标准标识符高位
				can0_writeByte(RXM0SIDL, 0xE0);//配置验收屏蔽寄存器n标准标识符低位
				maskh = (arg & 0xFF00) >> 8;
				maskl = arg & 0x00FF;
				can0_writeByte(RXF0SIDH, maskh);//配置验收滤波寄存器n标准标识符高位
				can0_writeByte(RXF0SIDL, maskl);//配置验收滤波寄存器n标准标识符低位
				h = can0_readByte(RXF0SIDH);
				l = can0_readByte(RXF0SIDL);
				if ((h << 8 | l) == arg)
					printk("Set CAN0 mask ID: 0x%x\n", (unsigned int)arg >> 5);
			}
			can0_writeByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//将MCP2515设置为正常模式,退出配置模式
			temp=can0_readByte(CANSTAT);//读取CAN状态寄存器的值
//			printk("temp = %x\n", temp);
			if(OPMODE_NORMAL!=(temp&&0xE0))//判断MCP2515是否已经进入正常模式
			{
				can0_writeByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//再次将MCP2515设置为正常模式,退出配置模式
			} else {
				printk("can0 mode: normal mode\n");
			}
			break;	
		//set ID
		case SETID:
			//set CNF1,SJW=00,长度为1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
			printk("Set CAN0 ID\n");
			can0_writeByte(CNF1, baudrate);
			//set CNF2,SAM=0,在采样点对总线进行一次采样，PHSEG1=(2+1)TQ=3TQ,PRSEG=(0+1)TQ=1TQ
			can0_writeByte(CNF2,0x80|PHSEG1_3TQ|PRSEG_1TQ);
			//set CNF3,PHSEG2=(2+1)TQ=3TQ,同时当CANCTRL.CLKEN=1时设定CLKOUT引脚为时间输出使能位
			can0_writeByte(CNF3,PHSEG2_3TQ);
			txidh = (arg & 0xFF00) >> 8;
			txidl = arg & 0x00FF;
			can0_writeByte(TXB0SIDH, txidh);//发送缓冲器0标准标识符高位
			can0_writeByte(TXB0SIDL, txidl);//发送缓冲器0标准标识符低位
			h = can0_readByte(TXB0SIDH);
			l = can0_readByte(TXB0SIDL);
			if ((h << 8 | l) == arg)
				printk("Set CAN0 send ID: 0x%x\n", (unsigned int)arg >> 5);
			printk("id = %x\n", (h << 8 | l) >> 5);
			break;	
		case STATE:
			temp=can0_readByte(EFLG);//读取CAN状态寄存器的值
			printk("EFLG = %x", temp);
			break;
		case GETID:
			h = can0_readByte(TXB0SIDH);
			l = can0_readByte(TXB0SIDL);
			return (h << 8 | l) >> 5;
			break;
		default:
			break;
	}

	return 0;
}

int can0_probe(struct platform_device *pdev)
{
	struct mcp2515_st *mcp;
	int ret;

	mcp = kzalloc(sizeof(*mcp), GFP_KERNEL);
	if (!mcp) {
		ret = -ENOMEM;
		goto alloc_led_error;
	}

	mcp->flag = CLOSE;

	spin_lock_init(&mcp->lock);

	mcp->mcp2515_ops.open = can0_open;
	mcp->mcp2515_ops.release = can0_close;
	mcp->mcp2515_ops.write = can0_write;
	mcp->mcp2515_ops.read = can0_read;
	mcp->mcp2515_ops.unlocked_ioctl = can0_ioctl;

	mcp->misc.minor = MISC_DYNAMIC_MINOR;
	mcp->misc.name = pdev->name;
	mcp->misc.fops = &mcp->mcp2515_ops;
																			
	ret = misc_register(&mcp->misc);
	if (ret) {
		goto register_misc_error;
	}

	can0_init(CAN_500Kbps);

	platform_set_drvdata(pdev, mcp);
																									
	return 0;

register_misc_error:
	kfree(mcp);	
alloc_led_error:
	return ret;
}

int can0_remove(struct platform_device *pdev)
{
	struct mcp2515_st *mcp;

	mcp = platform_get_drvdata(pdev);
	MCP2515_Reset();
	misc_deregister(&mcp->misc);
	kfree(mcp);	
	return 0;
}

struct platform_device_id can0_id_table[] = {
	{"can0", 123},
	{},
};

struct platform_driver can0_drv = {
	.probe = can0_probe,
	.remove = can0_remove,
	.driver = {
		.name = "can0",
	},
	.id_table = can0_id_table,
};

static __init int CAN0_init(void)
{
	return platform_driver_register(&can0_drv);
}

static __exit void can0_exit(void)
{
	platform_driver_unregister(&can0_drv);
}

module_init(CAN0_init);
module_exit(can0_exit);

MODULE_AUTHOR("horotororensu <han.liu@keluofeite.com>");
MODULE_DESCRIPTION("Microchip 2515 CAN driver for N32926");

MODULE_LICENSE("GPL");
