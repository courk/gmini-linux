#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include <argp.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/xattr.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <lkl.h>
#include <lkl_host.h>
#include <pthread.h>
#include <poll.h>
#include <sys/syscall.h>
#include <mtd/mtd-user.h>

#define KERNEL_CMDLINE "nandsim.first_id_byte=0x98 nandsim.second_id_byte=0xda nandsim.third_id_byte=0x90 nandsim.fourth_id_byte=0x15 nandsim.parts=14,8"

static const char doc_fsfuzz[] = "YAFFS2 fuzzing program";

static struct argp_option options[] = {
	{"enable-printk", 'p', 0, 0, "show Linux printks"},
	{0},
};

static struct cl_args {
	int printk;
} cla;

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
	struct cl_args *cla = state->input;

	switch (key) {
	case 'p':
		cla->printk = 1;
		break;
	default:
		return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp argp_fsfuzz = {
	.options = options,
	.parser = parse_opt,
	.doc = doc_fsfuzz,
};

/*
Use the filesytem in
various ways
 */
static void activity(char *mpoint) {

	char *foo_bar_baz;
	char *foo_baz;
	char *xattr;
	char *hln;
	char *sln;
	int err;

	static int buf[8192];
	memset(buf, 0, sizeof(buf));

	err = asprintf(&foo_bar_baz, "%s/foo/bar/baz", mpoint);
	err = asprintf(&foo_baz, "%s/foo/baz", mpoint);
	err = asprintf(&xattr, "%s/foo/bar/xattr", mpoint);
	err = asprintf(&hln, "%s/foo/bar/hln", mpoint);
	err = asprintf(&sln, "%s/foo/bar/sln", mpoint);

	// opendir / readdir
	DIR *dir = (DIR *)lkl_opendir(mpoint, &err);
	if (dir) {
		lkl_readdir((struct lkl_dir *)dir);
		lkl_closedir((struct lkl_dir *)dir);
	}

	// open / mmap / read
	int fd = lkl_sys_open(foo_bar_baz, LKL_O_RDONLY, 0);
	if (fd >= 0) {
		void *mem = lkl_sys_mmap(NULL, 4096, PROT_READ, MAP_PRIVATE | MAP_POPULATE, fd, 0);

		if (mem != MAP_FAILED)
			lkl_sys_munmap((unsigned long)mem, 4096);

		lkl_sys_read(fd, (char *)buf, 11);
		lkl_sys_read(fd, (char *)buf, sizeof(buf));
		lkl_sys_close(fd);
	}

	// open / write / read
	fd = lkl_sys_open(foo_bar_baz, O_RDWR | O_TRUNC, 0777);
	if (fd >= 0) {
		lkl_sys_write(fd, (char *)buf, 517);
		lkl_sys_write(fd, (char *)buf, sizeof(buf));
		lkl_sys_fdatasync(fd);
		lkl_sys_fsync(fd);

		lkl_sys_lseek(fd, 0, SEEK_SET);
		lkl_sys_read(fd, (char *)buf, sizeof(buf));
		lkl_sys_lseek(fd, 1234, SEEK_SET);
		lkl_sys_read(fd, (char *)buf, 517);
		lkl_sys_close(fd);
	}

	// open / lseek / write / fallocate
	fd = lkl_sys_open(foo_bar_baz, O_RDWR | O_TRUNC, 0777);
	if (fd >= 0) {
		lkl_sys_lseek(fd, 1024 - 33, SEEK_SET);
		lkl_sys_write(fd, (char *)buf, sizeof(buf));
		lkl_sys_lseek(fd, 1024 * 1024 + 67, SEEK_SET);
		lkl_sys_write(fd, (char *)buf, sizeof(buf));
		lkl_sys_lseek(fd, 1024 * 1024 * 1024 - 113, SEEK_SET);
		lkl_sys_write(fd, (char *)buf, sizeof(buf));

		lkl_sys_lseek(fd, 0, SEEK_SET);
		lkl_sys_write(fd, (char *)buf, sizeof(buf));

		lkl_sys_fallocate(fd, 0, 0, 123871237);
		lkl_sys_fallocate(fd, 0, -13123, 123);
		lkl_sys_fallocate(fd, 0, 234234, -45897);
		lkl_sys_fallocate(fd, FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE, 0, 4243261);
		lkl_sys_fallocate(fd, FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE, -95713, 38447);
		lkl_sys_fallocate(fd, FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE, 18237, -9173);

		lkl_sys_close(fd);
	}

	// rename
	lkl_sys_rename(foo_bar_baz, foo_baz);

	// stat
	struct lkl_stat stat;
	memset(&stat, 0, sizeof(stat));
	lkl_sys_stat(foo_baz, &stat);

	// chmod / chown
	lkl_sys_chmod(foo_baz, 0000);
	lkl_sys_chmod(foo_baz, 1777);
	lkl_sys_chmod(foo_baz, 3777);
	lkl_sys_chmod(foo_baz, 7777);
	lkl_sys_chown(foo_baz, 0, 0);
	lkl_sys_chown(foo_baz, 1, 1);

	// unlink
	lkl_sys_unlink(foo_bar_baz);
	lkl_sys_unlink(foo_baz);

	// mknod
	lkl_sys_mknod(foo_baz, 0777, LKL_MKDEV(0, 0));

	// xattr
	char buf2[113];
	memset(buf2, 0, sizeof(buf2));
	lkl_sys_listxattr(xattr, buf2, sizeof(buf2));
	lkl_sys_removexattr(xattr, "user.mime_type");
	lkl_sys_setxattr(xattr, "user.md5", buf2, sizeof(buf2), XATTR_CREATE);
	lkl_sys_setxattr(xattr, "user.md5", buf2, sizeof(buf2), XATTR_REPLACE);

	// link
	lkl_sys_readlink(sln, buf2, sizeof(buf2));
}

/*
Flash the virtual nand
created by the nandsim driver
 */
static int flash_nand(char *buffer, int length) {
	mtd_info_t mtd_info;
	erase_info_t ei;
	struct mtd_write_req ops;
	int page_index;

	int fd = lkl_sys_open("/dev/mtd0", O_RDWR, 0);

	if (lkl_sys_ioctl(fd, MEMGETINFO, (long unsigned int)&mtd_info) < 0) {
		printf("Cannot get nand info\n");
		goto error;
	}

	/*
	Erase
	 */
	ei.length = mtd_info.erasesize;
	for (ei.start = 0; ei.start < mtd_info.size; ei.start += ei.length)
	{
		if (lkl_sys_ioctl(fd, MEMERASE, (long unsigned int)&ei) < 0) {
			printf("Cannot MEMERASE\n");
			goto error;
		}
	}

	if (lkl_sys_lseek(fd, 0, SEEK_SET) < 0)
		goto error;

	/*
	Flash
	 */
	for (page_index = 0; page_index < length / 0x840; page_index++) {
		ops.start = page_index * 0x800;
		ops.len = 0x800;
		ops.ooblen = 0x40;
		ops.usr_data = (uint64_t)(unsigned long)(buffer + page_index * 0x840);
		ops.usr_oob = (uint64_t)(unsigned long)(buffer + page_index * 0x840 + 0x800);
		ops.mode = MTD_OPS_PLACE_OOB;
		if (lkl_sys_ioctl(fd, MEMWRITE, (long)&ops) < 0) {
			printf("Cannot write to the nand\n");
			goto error;
		}
	}

	lkl_sys_close(fd);

	return 0;

error:
	return -1;
}

static uint32_t new_encode_dev(unsigned int major, unsigned int minor)
{
	return (minor & 0xff) | (major << 8) | ((minor & ~0xff) << 12);
}


int lkl_encode_dev_from_sysfs(const char *sysfs_path, uint32_t *pdevid)
{
	int ret;
	long fd;
	int major, minor;
	char buf[16] = { 0, };
	char *bufptr;

	fd = lkl_sys_open(sysfs_path, LKL_O_RDONLY, 0);
	if (fd < 0) {
		printf("...\n");
		return fd;
	}

	ret = lkl_sys_read(fd, buf, sizeof(buf));
	if (ret < 0)
		goto out_close;

	if (ret == sizeof(buf)) {
		ret = -LKL_ENOBUFS;
		goto out_close;
	}

	bufptr = strchr(buf, ':');
	if (bufptr == NULL) {
		ret = -LKL_EINVAL;
		goto out_close;
	}
	bufptr[0] = '\0';
	bufptr++;

	major = atoi(buf);
	minor = atoi(bufptr);

	*pdevid = new_encode_dev(major, minor);
	ret = 0;

out_close:
	lkl_sys_close(fd);

	return ret;
}

int main(int argc, char **argv)
{
	long err;
	uint32_t dev;
	char image_buffer[200 * 1024];
	size_t size;

	if (argp_parse(&argp_fsfuzz, argc, argv, 0, 0, &cla) < 0)
		return -1;

	if (!cla.printk)
		lkl_host_ops.print = NULL;

	__AFL_INIT();

	lkl_start_kernel(&lkl_host_ops, 100 * 1024 * 1024,
	                 KERNEL_CMDLINE);

	/*
	Make /dev/mtdblock0 accessible
	 */
	lkl_mount_fs("sysfs");

	err = lkl_sys_access("/dev", LKL_S_IRWXO);
	if (err < 0) {
		if (err == -LKL_ENOENT)
			err = lkl_sys_mkdir("/dev", 0700);
		if (err < 0)
			return err;
	}

	err = lkl_encode_dev_from_sysfs("/sysfs/block/mtdblock0/dev", &dev);

	if (err < 0) {
		fprintf(stderr, "can't lkl_encode_dev_from_sysfs(mtdblock0): %s\n", lkl_strerror(err));
		return err;
	}

	err = lkl_sys_mknod("/dev/mtdblock0", LKL_S_IFBLK | 0600, dev);
	if (err < 0) {
		fprintf(stderr, "can't mknod: %s\n", lkl_strerror(err));
		return err;
	}

	/*
	Make /dev/mtd0 accessible
	 */

	err = lkl_encode_dev_from_sysfs("/sysfs/class/mtd/mtd0/dev", &dev);

	if (err < 0) {
		fprintf(stderr, "can't lkl_encode_dev_from_sysfs(mtd0): %s\n", lkl_strerror(err));
		return err;
	}

	err = lkl_sys_mknod("/dev/mtd0", LKL_S_IFCHR | 0600, dev);
	if (err < 0) {
		fprintf(stderr, "can't mknod: %s\n", lkl_strerror(err));
		return err;
	}

	/*
	Create /mnt
	 */
	err = lkl_sys_access("/mnt", LKL_S_IRWXO);
	if (err < 0) {
		if (err == -LKL_ENOENT)
			err = lkl_sys_mkdir("/mnt", 0700);
		if (err < 0) {
			fprintf(stderr, "can't mkdir: %s\n", lkl_strerror(err));
			return err;
		}
	}

	/*
	AFL fuzz loop
	 */
	while (__AFL_LOOP(1000)) {

		/*
		Flash the virtual nand with an image read
		from stdin
		 */
		size = read(0, image_buffer, 200 * 1024);
		if (flash_nand(image_buffer, size) < 0) {
			printf("Cannot flash the nand\n");
			goto out;
		}

		/*
		Attempt to mount the image
		 */
		err = lkl_sys_mount("/dev/mtdblock0", "/mnt/", "yaffs2", 0, "tags-ecc-off");
		if (err) {
			fprintf(stderr, "can't mount disk: %s\n", lkl_strerror(err));
			goto out;
		}

		/*
		Use the filesystem
		 */
		activity("/mnt/");

		/*
		Umount the filesystem
		 */
		lkl_sys_umount("/mnt", 0);
	}

out:
	lkl_sys_halt();

	return 0;
}
