
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>


  /* needed for virt_to_phys() */
#include <asm/io.h> // virt_to_phys()
#include <linux/kernel.h>	/* printk() */
#include <linux/kdev_t.h>
#include <asm/page.h>
#include <asm/sections.h>
#include <linux/cdev.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#include <asm/uaccess.h>
#include <media/blackfin/bfin_capture.h>

#define FRAME_MAX	1310720


#define ACD_MAGIC		'C'
#define ACD_GET_REG		_IOWR(ACD_MAGIC, 0, struct camera_reg *)
#define ACD_SET_REG		_IOW(ACD_MAGIC, 1, struct camera_reg *)

#define ACD_GET_LIGHT		_IOWR(ACD_MAGIC, 2, u16)
#define ACD_SET_LIGHT		_IOW(ACD_MAGIC, 3, u16)

#define ACD_START_STOP_STREAM		_IOW(ACD_MAGIC, 4, u32)
#define ACD_GET_BUFFER_SYNC			_IOWR(ACD_MAGIC, 5, u32)
#define ACD_RELEASE_BUFFER_SYNC 	_IOWR(ACD_MAGIC, 6, u32)
#define ACD_SET_NEW_FORMAT			_IOWR(ACD_MAGIC, 7, u32)
#define ACD_SET_CONTRAST_LIMIT0		_IOWR(ACD_MAGIC, 8, u32)
#define ACD_SET_CONTRAST_LIMIT1		_IOWR(ACD_MAGIC, 9, u32)

#define PPI_CONTROL			0xFFC01000	/* PPI Control Register			*/
#define PPI_STATUS			0xFFC01004	/* PPI Status Register			*/
#define PPI_COUNT			0xFFC01008	/* PPI Transfer Count Register	*/
#define PPI_DELAY			0xFFC0100C	/* PPI Delay Count Register		*/
#define PPI_FRAME			0xFFC01010	/* PPI Frame Length Register	*/

#define PORTF_FER			0xFFC03200	/* Port F Function Enable Register (Alternate/Flag*)	*/
#define PORTG_FER			0xFFC03204	/* Port G Function Enable Register (Alternate/Flag*)	*/
#define PORTH_FER			0xFFC03208	/* Port H Function Enable Register (Alternate/Flag*)	*/
#define BFIN_PORT_MUX		0xFFC0320C	/* Port Multiplexer Control Register*/

#define PORTF_MUX           0xFFC03210      /* Port F mux control */
#define PORTG_MUX           0xFFC03214      /* Port G mux control */
#define PORTH_MUX           0xFFC03218      /* Port H mux control */

#define DMA_TC_CNT			0xFFC00B10	/* Traffic Control Current Counts Register*/

#define DMA0_START_ADDR			0xFFC00C04	/* DMA Channel 0 Start Address Register					*/
#define DMA0_CONFIG				0xFFC00C08	/* DMA Channel 0 Configuration Register					*/
#define DMA0_X_COUNT			0xFFC00C10	/* DMA Channel 0 X Count Register						*/
#define DMA0_X_MODIFY			0xFFC00C14	/* DMA Channel 0 X Modify Register						*/
#define DMA0_Y_COUNT			0xFFC00C18	/* DMA Channel 0 Y Count Register						*/
#define DMA0_Y_MODIFY			0xFFC00C1C	/* DMA Channel 0 Y Modify Register						*/

#define ACD_CAPTURE_DRV_NAME        "acd6_camera"
#define acd6d_MIN_NUM_BUF        2

#define PG11		(1 << 11)




struct acd6c_format {
	u16 l;
	u16 t;
	u16 w;
	u16 h;
	u8 bpp; /* bits per pixel */
};

static const struct acd6c_format fmts[] = {
	{
		.bpp        = 8,
		.l 			= 20,
		.t 			= 12,
		.w 			= 1280,
		.h   		= 1024,
	},

	{
		.bpp	= 8,
		.l 		= 320,
		.t 		= 256,
		.w 		= 640,
		.h 		= 512,
	}
};

struct vd_buffer {
	void *vd_buf;
	//unsigned long vd_buf_phys;
	dma_addr_t vd_buf_phys;
	struct vd_buffer *next;
	volatile int frame_counter;
	volatile u8 lock;
	volatile u8 phase;
	u8 num;
};

struct acd6c_device
{
	unsigned short ppi_control;

	const struct ppi_info *ppi_info;
	/* ppi interface */
	struct ppi_if *ppi;
	/* bits per pixel*/
	int bpp;

	spinlock_t lock;
	spinlock_t lock_irq;

	/* used to wait ppi to complete one transfer */
	struct completion comp;

	bool stream;

	volatile int frame_complete;

	struct _camera_setup {
		u16 left;
		u16 top;
		u16 width;
		u16 height;

		u16 global_gain;
		u16 global_offset;

		u8 mirrored;
	} cam_s;

	struct vd_buffer buf0, buf1, temp_buf, *current_buf, *prev_buf;
	u32 detect_thr[2];
};

struct cam_control {
	int cam_number;
	u8 *buf_ptr;
};

struct acd6c_device * d;

u8 current_camera = 0;
static u8 get_frame(struct cam_control * cc);
static void print_ppi_config();
static irqreturn_t acd6c_isr(int irq, void *dev_id);
static void set_ppi (struct acd6c_format *f);
//static void switch_camera( u8 num );





static int acd6c_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int acd6c_close(struct inode *inode, struct file *file)
{
	d->ppi->ops->stop(d->ppi);
	return 0;
}


static 	long acd6c_ioctl (struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct acd6c_format f;
	struct cam_control cam_c;
	struct ppi_if *ppi = d->ppi;
	//printk("arg=%x, cmd=%x\n", arg, cmd);
	switch (cmd) {


		case ACD_SET_REG:
			break;
		case ACD_GET_REG:
			break;
		case ACD_SET_NEW_FORMAT:

			set_ppi((struct acd6c_format *)arg);

			break;
		case ACD_START_STOP_STREAM:
			if (arg)
			{
				d->stream = true;
				ppi->ops->start(ppi);

			} else
			{
				d->stream = false;
				ppi->ops->stop(ppi);
			}

			break;


		case ACD_GET_BUFFER_SYNC:
		{
			get_frame((struct cam_control *)arg);
			break;
		}

		case ACD_RELEASE_BUFFER_SYNC:
		{
			if (arg == 0) {
				d->buf0.lock = 0;
			}
			if (arg == 1) {
				d->buf1.lock = 0;
			}


			break;
		}
		case ACD_SET_CONTRAST_LIMIT1:
			d->detect_thr[0] = arg;
					break;
		case ACD_SET_CONTRAST_LIMIT0:
			d->detect_thr[1] = arg;
			break;

		default:
			printk("NO SUCH CMD!\n");
			break;
	}


	return 0;
}

static u8 get_frame(struct cam_control * cc)
{
	int curr_frame = 0;
	int last_fr_num = 0;
	volatile int *frame_counter;
	int tmo = 100, i;

	if (cc == NULL) {
		printk("CC is NULL. control message error\n");
		return 1;
	}
//


	if ((cc->cam_number == 0)&&(d->buf0.lock)){
		d->stream = true;
		cc->buf_ptr = (u8 *)d->buf0.vd_buf;
		//d->buf0.lock = 0;
		return 0;
	}

	if ((cc->cam_number == 1)&&(d->buf1.lock)) {
		d->stream = true;
		cc->buf_ptr = (u8 *)d->buf1.vd_buf;
		//d->buf1.lock = 0;
		return 0;
	}

	if (cc->cam_number == 3)	// 0 in snapshot mode
	{
		d->stream = false;
		cc->buf_ptr = (u8 *)d->buf0.vd_buf;
		tmo = 1000;	// 1 sec timeout

		d->ppi->ops->stop(d->ppi);
		set_gpio_data(GPIO_PH4, 0);
		d->ppi->ops->update_addr(d->ppi, d->buf0.vd_buf_phys);
		d->frame_complete = 0;
		d->ppi->ops->start(d->ppi);

		while ((d->frame_complete == 0) && (--tmo)) udelay(1000);

		if (!tmo) {
			printk("Timeout in %s\n", __func__);
			cc->buf_ptr = NULL;
			return 1;
		}
	}
	if (cc->cam_number == 4)	// 1 in snapshot mode
	{
		d->stream = false;
		cc->buf_ptr = (u8 *)d->buf1.vd_buf;
		tmo = 1000;	// 1 sec timeout

		d->ppi->ops->stop(d->ppi);
		set_gpio_data(GPIO_PH4, 1);
		d->ppi->ops->update_addr(d->ppi, d->buf1.vd_buf_phys);
		d->frame_complete = 0;
		d->ppi->ops->start(d->ppi);

		while ((d->frame_complete == 0) && (--tmo)) udelay(1000);

		if (!tmo) {
			printk("Timeout in %s\n", __func__);
			cc->buf_ptr = NULL;
			return 1;
		}
	}
//
//
//	printk("TMO=%d %s\n",tmo, __func__);
	return 0;
}


const struct file_operations acd6c_fops = {
        .owner                	= THIS_MODULE,
        .open                	= acd6c_open,
        .release				= acd6c_close,
        .unlocked_ioctl 		= acd6c_ioctl,
};

static struct miscdevice acd6_misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = ACD_CAPTURE_DRV_NAME,
		.fops = &acd6c_fops
};

static const unsigned short ppi_req[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};


static const struct ppi_info ppi_info = {
	.name = "ppi",
	.dma_ch = 0,
	.irq_err = BFIN_IRQ(6),
	.base = PPI_CONTROL,
	.pin_req = ppi_req,
};

static void print_ppi_config() {


#define DMA0_START_ADDR			0xFFC00C04	/* DMA Channel 0 Start Address Register					*/
#define DMA0_CONFIG				0xFFC00C08	/* DMA Channel 0 Configuration Register					*/
#define DMA0_X_COUNT			0xFFC00C10	/* DMA Channel 0 X Count Register						*/
#define DMA0_X_MODIFY			0xFFC00C14	/* DMA Channel 0 X Modify Register						*/
#define DMA0_Y_COUNT			0xFFC00C18	/* DMA Channel 0 Y Count Register						*/
#define DMA0_Y_MODIFY			0xFFC00C1C	/* DMA Channel 0 Y Modify Register						*/


	printk("PPI_CONTROL:%04X\n", bfin_read16(PPI_CONTROL));
	printk("PPI_STATUS:%04X\n", bfin_read16(PPI_STATUS));
	printk("PPI_COUNT:%04X\n", bfin_read16(PPI_COUNT));
	printk("PPI_DELAY:%04X\n", bfin_read16(PPI_DELAY));
	printk("PPI_FRAME:%04X\n", bfin_read16(PPI_FRAME));

	printk("PORTF_FER:%04X\n", bfin_read16(PORTF_FER));
	printk("PORTG_FER:%04X\n", bfin_read16(PORTG_FER));
	printk("PORTH_FER:%04X\n", bfin_read16(PORTH_FER));

	printk("PORTF_MUX:%04X\n", bfin_read16(PORTF_MUX));
	printk("PORTG_MUX:%04X\n", bfin_read16(PORTG_MUX));
	printk("PORTH_MUX:%04X\n", bfin_read16(PORTH_MUX));

	printk("DMA0_START_ADDR:%08X\n", bfin_read32(DMA0_START_ADDR));
	printk("DMA0_CONFIG:%04X\n", bfin_read16(DMA0_CONFIG));
	printk("DMA0_X_COUNT:%04X\n", bfin_read16(DMA0_X_COUNT));
	printk("DMA0_X_MODIFY:%04X\n", bfin_read16(DMA0_X_MODIFY));
	printk("DMA0_Y_COUNT:%04X\n", bfin_read16(DMA0_Y_COUNT));
	printk("DMA0_Y_MODIFY:%04X\n", bfin_read16(DMA0_Y_MODIFY));
	printk("DMA_TC_CNT:%04X\n", bfin_read16(DMA_TC_CNT));
	printk("BFIN_PORT_MUX:%04X\n", bfin_read16(BFIN_PORT_MUX));

	return;
}

static void set_ppi (struct acd6c_format *f)
{
	struct ppi_if *ppi = d->ppi;
	struct ppi_params params;
	int retval = 0;
	printk("Set capture params [%d %d %d %d]\n", f->l, f->t, f->w, f->h);
	d->ppi_control = (POLC | PACK_EN | DLEN_8 | XFR_TYPE | 0x0020);
	params.width = f->w;
	params.height = f->h;
	params.bpp = f->bpp;
	params.ppi_control = d->ppi_control;
	retval = ppi->ops->set_params(ppi, &params);
	if (retval < 0) {
		printk("Error in setting ppi params\n");
		ppi->ops->detach_irq(ppi);
	}
	return;
}


u32 estimate_contrast(u8 *line, u16 len)
{
	u32 contr = 0;
	u16 i = 1;

	for (; i < len - 1; i++) {
		contr += abs(line[i] - line[i + 1]);
	}

	return contr;
}

u32 check_finger(u16 w, u16 h, u8 *buff) {

	u32 contrast = 0;
	//*c1 = 0;

	u32 IW = w;
	u32 IH = h;

	u8 * line1, * line2, * line3, * line4, * line5, * line6, * line7 = NULL;

	line1 = &buff[IW*128];
	line2 = &buff[IW*256];
	line3 = &buff[IW*384];
	line4 = &buff[IW*512];
	line5 = &buff[IW*640];
	line6 = &buff[IW*768];
	line7 = &buff[IW*896];

	contrast += estimate_contrast(line1, IW);
	contrast += estimate_contrast(line2, IW);
	contrast += estimate_contrast(line3, IW);
	contrast += estimate_contrast(line4, IW);
	contrast += estimate_contrast(line5, IW);
	contrast += estimate_contrast(line6, IW);
	contrast += estimate_contrast(line7, IW);

	contrast >>= 2;

	return contrast;
}

static irqreturn_t acd6c_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = d->ppi;
	u8 *ptr = NULL;
	u32 detect_score = 0;
	spin_lock(&d->lock);


	if (d->buf0.lock) d->buf0.lock--;

	if (d->buf1.lock) d->buf1.lock--;

	ppi->ops->stop(ppi);

	u8 cap_frame_number = d->current_buf->num;

	if (d->stream) {
		if (d->current_buf->num == 0)
		{
			if (d->buf0.phase == 0) // capture finished
			{
				detect_score = check_finger(1280, 1024, d->temp_buf.vd_buf);
	//			printk("0:%d\n", detect_score);
				if ((detect_score > d->detect_thr[0])&&(d->buf0.lock == 0)) // detected fingerprint
				{
					ppi->ops->update_addr(ppi, d->buf0.vd_buf_phys);
					d->buf0.phase = 1;
					set_gpio_data(GPIO_PH4, 0);
					printk("F0:%d\n", detect_score);
				}
				else
				{

					d->current_buf = d->current_buf->next;
					ppi->ops->update_addr(ppi, d->temp_buf.vd_buf_phys);
					set_gpio_data(GPIO_PH4, 1);
				}
			}
			else
			{
				// mark other buffer
				d->current_buf = d->current_buf->next;
				ppi->ops->update_addr(ppi, d->temp_buf.vd_buf_phys);
				d->buf0.phase = 0;
				//printk("F0-2\n");
				d->buf0.lock = 60;
				set_gpio_data(GPIO_PH4, 1);
			}
		}
		else if (d->current_buf->num == 1)
		{
			if (d->buf1.phase == 0) // capture finished
			{
				detect_score = check_finger(1280, 1024, d->temp_buf.vd_buf);
	//			printk("1:            %d\n", detect_score);
				if ((detect_score > d->detect_thr[1]) && (d->buf1.lock == 0)) // detected fingerprint
				{
					ppi->ops->update_addr(ppi, d->buf1.vd_buf_phys);
					d->buf1.phase = 1;
					set_gpio_data(GPIO_PH4, 1);
					printk("F1:%d\n", detect_score);
				}
				else
				{

					d->current_buf = d->current_buf->next;
					ppi->ops->update_addr(ppi, d->temp_buf.vd_buf_phys);
					set_gpio_data(GPIO_PH4, 0);
				}
			}
			else
			{
				// mark other buffer
				d->current_buf = d->current_buf->next;
				ppi->ops->update_addr(ppi, d->temp_buf.vd_buf_phys);
				d->buf1.phase = 0;
				//printk("F1-2\n");
				d->buf1.lock = 60;
				set_gpio_data(GPIO_PH4, 0);
			}
		}

		ppi->ops->start(ppi);
	}

	spin_unlock(&d->lock);

	d->frame_complete = 1;

	return IRQ_HANDLED;
}


static int __devinit acd6c_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct ppi_if *ppi;
	// Alloccate device
	d = kmalloc(sizeof(struct acd6c_device), GFP_KERNEL);
	// Init spinlock
	spin_lock_init(&d->lock);
	// Alloc frame memory

	d->buf0.vd_buf = kmalloc(FRAME_MAX, GFP_DMA);
	d->buf0.vd_buf_phys = virt_to_phys(d->buf0.vd_buf);
	d->buf1.vd_buf = kmalloc(FRAME_MAX, GFP_DMA);
	d->buf1.vd_buf_phys = virt_to_phys(d->buf1.vd_buf);
	d->temp_buf.vd_buf = kmalloc(FRAME_MAX, GFP_DMA);
	d->temp_buf.vd_buf_phys = virt_to_phys(d->temp_buf.vd_buf);


	d->buf0.num = 0;
	d->buf1.num = 1;


	if ((!d->buf0.vd_buf) || (!d->buf1.vd_buf) || (!d->temp_buf.vd_buf)) {
		printk("Unable to alloc DMA memory\n");
		return -ENOMEM;
	}



	retval = misc_register(&acd6_misc);

	if (retval)
		printk("failed to register misc device\n");

	d->ppi_info = &ppi_info;

	d->ppi = ppi_create_instance(d->ppi_info);

	if (!d->ppi) {
		printk("Unable to create ppi\n");
		return -ENODEV;
	}

	ppi = d->ppi;

	ppi->priv = d;
	// setup PPI IRQ
	retval = ppi->ops->attach_irq(ppi, acd6c_isr);
	if (retval < 0) {
		printk("Error in attaching interrupt handler\n");
		return -EFAULT;
	}
	/* set ppi params */
	set_ppi(&fmts[0]);
	// update DMA address

	// setup led pin
	bfin_gpio_request(GPIO_PH4, "LED");
	bfin_gpio_direction_output(GPIO_PH4, 1);
	// setup select pin

	d->buf0.frame_counter = 0;
	d->buf1.frame_counter = 0;

	d->buf0.lock = 0;
	d->buf1.lock = 0;

	d->buf0.phase = 0;
	d->buf1.phase = 0;

	d->buf0.next = &d->buf1;
	d->buf1.next = &d->buf0;

	d->current_buf = &d->buf0;

	d->detect_thr[0] = 8000;
	d->detect_thr[1] = 8000;
	ppi->ops->update_addr(ppi, d->temp_buf.vd_buf_phys);
	set_gpio_data(GPIO_PH4, d->current_buf->num);
//	ppi->ops->start(d->ppi);

	return 0;
}



static int __devexit acd6c_remove(struct platform_device *pdev)
{
	//Remove instance and IRQ
	d->ppi->ops->detach_irq(d->ppi);
	ppi_delete_instance(d->ppi);
	// Free frame memory
	if (d->buf0.vd_buf)
		kfree(d->buf0.vd_buf);
	if (d->buf1.vd_buf)
		kfree(d->buf1.vd_buf);
	// Free device memory
	kfree(d);
	// Remove device
	misc_deregister(&acd6_misc);
	return 0;
}

static struct platform_driver acd6c_driver = {
	.driver = {
		.name   = ACD_CAPTURE_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = acd6c_probe,
	.remove = __devexit_p(acd6c_remove),
};

static __init int acd6c_init(void)
{
 	return platform_driver_register(&acd6c_driver);
}

static __exit void acd6c_exit(void)
{
	return platform_driver_unregister(&acd6c_driver);
}

module_init(acd6c_init);
module_exit(acd6c_exit);

MODULE_DESCRIPTION("blackfin video capture driver");
MODULE_AUTHOR("Iam");
MODULE_LICENSE("GPL");
