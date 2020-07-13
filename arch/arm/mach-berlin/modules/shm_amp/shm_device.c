/********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
 * on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 ********************************************************************************/


/*******************************************************************************
  System head files
*/

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */

/*******************************************************************************
  Local head files
*/

#include "shm_type.h"
#include "memory_engine.h"

/*******************************************************************************
  Device API
*/

int shm_device_create(shm_device_t ** device,
		      uint base, uint size, uint threshold)
{
	int res;
	shm_device_t *new_device;

	shm_debug("shm_device_create start.\n");

	if (device == NULL)
		return -EINVAL;

	new_device = kmalloc(sizeof(shm_device_t), GFP_KERNEL);
	if (new_device == NULL)
		return -ENOMEM;

	memset(new_device, 0, sizeof(shm_device_t));
	new_device->m_base = base;
	new_device->m_size = size;
	new_device->m_threshold = threshold;

	res = memory_engine_create(&new_device->m_engine,
				   new_device->m_base,
				   new_device->m_size, new_device->m_threshold);
	if (res != 0) {
		shm_error("memory_engine_create failed. (%d)\n", res);
		return res;
	}

	shm_trace("memory size (bytes)                 = 0x%08X\n",
		  new_device->m_size);
	shm_trace("memory threshold (bytes)            = 0x%08X\n",
		  new_device->m_threshold);
	shm_trace("memory base phys addr               = 0x%08X\n",
		  new_device->m_base);

	*device = new_device;

	return 0;
}

int shm_device_destroy(shm_device_t ** device)
{
	int res;

	shm_debug("shm_device_destroy start.\n");

	if (device == NULL)
		return -EINVAL;

	res = memory_engine_destroy(&((*device)->m_engine));
	if (res != 0) {
		shm_error("memory_engine_destroy failed. (%d)\n", res);
		return res;
	}

	kfree(*device);
	*device = NULL;

	shm_trace("shm_device_destroy OK.\n");

	return 0;
}

int shm_device_get_meminfo(shm_device_t * device, pMV_SHM_MemInfo_t pInfo)
{
	shm_debug("shm_device_get_meminfo start.\n");

	if ((device == NULL) || (pInfo == NULL))
		return -EINVAL;

	pInfo->m_totalmem = device->m_engine->m_size;
	pInfo->m_usedmem = device->m_engine->m_size_used;
	pInfo->m_freemem = device->m_engine->m_size_free;

	pInfo->m_peak_usedmem = device->m_engine->m_peak_usedmem;
	pInfo->m_max_freeblock = device->m_engine->m_max_freeblock;
	pInfo->m_min_freeblock = device->m_engine->m_min_freeblock;
	pInfo->m_max_usedblock = device->m_engine->m_max_usedblock;
	pInfo->m_min_usedblock = device->m_engine->m_min_usedblock;
	pInfo->m_num_freeblock = device->m_engine->m_num_freeblock;
	pInfo->m_num_usedblock = device->m_engine->m_num_usedblock;

	return 0;
}

int shm_device_get_baseinfo(shm_device_t * device, pMV_SHM_BaseInfo_t pInfo)
{
	shm_debug("shm_device_get_baseinfo start.\n");

	if ((device == NULL) || (pInfo == NULL))
		return -EINVAL;

	pInfo->m_size = device->m_size;
	pInfo->m_threshold = device->m_threshold;
	pInfo->m_base_physaddr = device->m_base;

	return 0;
}


int shm_device_get_node_info(shm_device_t *device,
				shm_driver_operation_t *op)
{
	memory_node_t *tmp;
	if (device == NULL)
		return -1;
	mutex_lock(&(device->m_engine->m_mutex));
	tmp = memory_engine_lookup_shm_node_for_size(
		&(device->m_engine->m_shm_root), op->m_param1);

	op->m_param2 = 0;
	if (tmp == NULL) {
		printk("shm_device_get_node_info fail"
			" physaddress[%08lx]\n",op->m_param1);
		mutex_unlock(&(device->m_engine->m_mutex));
		return -1;
	}

	op->m_param1 = tmp->m_phyaddress;
	op->m_param2 = MEMNODE_ALIGN_SIZE(tmp);
	mutex_unlock(&(device->m_engine->m_mutex));
	return 0;
}

int shm_device_check_test(shm_device_t *device,
				shm_check_test_t *op)
{
	int i = 0;
	int res = 0;
	memory_node_t *node = NULL;
	user_node_t *usertmp = NULL;

	if (device == NULL)
		return -1;
	mutex_lock(&(device->m_engine->m_mutex));
	node = memory_engine_lookup_shm_node_for_size(
		&device->m_engine->m_shm_root,
		op->offset + device->m_engine->m_base);
	if (node == NULL) {
		op->node_alive = 0;
	} else {
		op->node_alive = 1;
		switch (op->cmd) {
		case SHM_TEST_GET_INFO:
			op->takeover_flag = node->b_takeover;
			op->physical = node->m_phyaddress;
			op->malloc_refcount = node->m_task_free_flag;
			op->malloc_taskid = node->m_taskid;

			list_for_each_entry(usertmp, &node->m_user_list,
					m_list) {
				op->user_taskid[i] = usertmp->m_taskid;
				i++;
				if (i == 5)
					break;
			}
			res = 0;
			break;
		}
	}
	mutex_unlock(&(device->m_engine->m_mutex));
	return res;
}

int shm_device_add_reference_count(shm_device_t *device,
					size_t offset)
{
	pid_t taskid;
	size_t phyaddress;
	int res = 0;

	memory_node_t *node = NULL;
	user_node_t *tmp = NULL;
	struct task_struct *grptask = NULL;

	if (device == NULL) {
		shm_error("shm_device_free device is NULL\n");
		return -EINVAL;
	}

	phyaddress = offset + device->m_engine->m_base;
	mutex_lock(&device->m_engine->m_mutex);
	node = memory_engine_lookup_shm_node_for_size(
			&device->m_engine->m_shm_root,
			phyaddress);
	if (node == NULL) {
		printk("shm_device_add_reference_count fail 1"
			" physaddress[%08x]\n", phyaddress);
		res = -1;
		goto EXIT;
	}

	if (node->b_takeover == 1) {
		res = 0;
		goto EXIT;
	}

	taskid = task_tgid_vnr(current);
	if (taskid == node->m_taskid) {
		if (node->memory_type == SHM_SECURE_CACHE ||
			node->memory_type == SHM_SECURE_NONCACHE) {
			node->m_refcount++;
			node->m_reference_count++;
		}
		res = 0;
		goto EXIT;
	}

	list_for_each_entry(tmp, &node->m_user_list, m_list) {
		if (tmp->m_taskid == taskid) {
			if (node->memory_type == SHM_SECURE_CACHE ||
				node->memory_type == SHM_SECURE_NONCACHE) {
				tmp->m_refcount++;
				node->m_reference_count++;
			}
			res = 0;
			goto EXIT;
		}
	}

	tmp = kmalloc(sizeof(user_node_t), GFP_KERNEL);
	if (tmp == NULL) {
		res = -ENOMEM;
		goto EXIT;
	}

	tmp->m_refcount = 1;
	tmp->m_threadid = task_pid_vnr(current);
	strncpy(tmp->m_threadname, current->comm, 16);
	tmp->m_taskid = task_tgid_vnr(current);
	grptask = pid_task(task_tgid(current), PIDTYPE_PID);
	if (NULL != grptask) {
		strncpy(tmp->m_taskname, grptask->comm, 16);
	} else {
		memset(tmp->m_taskname, 0, 16);
	}

	list_add(&tmp->m_list, &node->m_user_list);
	node->m_reference_count++;
	res = 0;

EXIT:
	mutex_unlock(&device->m_engine->m_mutex);
	return res;
}

int shm_device_show_detail(shm_device_t * device, struct seq_file *file)
{
	shm_debug("shm_device_show_detail start.\n");

	if ((device == NULL) || (file == NULL))
		return -EINVAL;

	/* Walk all nodes and print node info to buffer */
	return memory_engine_show(device->m_engine, file);
}

uint shm_device_allocate(shm_device_t * device, uint size, uint align, int mem_type)
{
	int res = 0;
	uint offset = 0;
	memory_node_t *node = NULL;
	memory_engine_t *engine = NULL;
	pid_t taskid = task_tgid_vnr(current);

	shm_debug("shm_device_allocate start. (%u, %u)\n", size, align);

	if (device == NULL)
		return ERROR_SHM_MALLOC_FAILED;

	engine = device->m_engine;
	if (NULL == engine)
		return ERROR_SHM_MALLOC_FAILED;

	res = memory_engine_allocate(device->m_engine, size, align, &node);
	if (res != 0) {
		memory_engine_dump_stat(device->m_engine);

		/* invoke lowmem killer to release less important task */
		if (NULL != device->m_shrinker)
			device->m_shrinker((void *)device, size);

		shm_error("memory_engine_allocate failed, size:%d pid:%d\n", size, taskid);
		return ERROR_SHM_MALLOC_FAILED;
	}

	offset = MEMNODE_ALIGN_ADDR(node) - device->m_engine->m_base;
	node->memory_type = mem_type;

	shm_debug("shm_device_allocate get offset (%u) OK\n", offset);

	return offset;
}

int shm_device_free(shm_device_t * device, uint offset)
{
	int res = 0;
	uint alignaddr = 0;
	pid_t taskid = task_tgid_vnr(current);

	shm_debug("shm_device_free start. (%u)\n", offset);

	if (device == NULL) {
		shm_error("shm_device_free device is NULL pid:%d\n", taskid);
		return -EINVAL;
	}

	alignaddr = device->m_engine->m_base + offset;

	res = memory_engine_free(device->m_engine, alignaddr);
	if (res != 0) {
		return res;
	}

	shm_debug("shm_device_free offset (%u) OK\n", offset);

	return 0;
}


int shm_device_cache(shm_device_t *device, uint cmd,
			shm_driver_operation_t op, int mem_type)
{
	int res = 0;
	char tag_clean[] = "clean";
	char tag_invalidate[] = "invalidate";
	char tag_cleanAndinvalidate[] = "clean and invalidate";
	char *ptr_tag;

	if (device == NULL) {
		shm_error("shm_device_free device is NULL\n");
		return -EINVAL;
	}

	if ((mem_type == SHM_CACHE) || (mem_type == SHM_SECURE_CACHE)) {
		res = memory_engine_cache(device->m_engine, cmd, op);
	} else { // non-cache
		res = -1;
		if (cmd == SHM_DEVICE_CMD_INVALIDATE) {
			ptr_tag = tag_invalidate;
		} else if (cmd == SHM_DEVICE_CMD_CLEAN) {
			ptr_tag = tag_clean;
		} else {
			ptr_tag = tag_cleanAndinvalidate;
		}
		printk("SHM noncache not need %s cache\n",ptr_tag);
	}
	return res;
}

int shm_device_takeover(shm_device_t* device, uint offset)
{
	int res = 0;
	uint alignaddr = 0;

	shm_debug("shm_device_takeover start (%u)\n", offset);

	if (NULL == device)
		return -EINVAL;

	alignaddr = device->m_engine->m_base + offset;

	res = memory_engine_takeover(device->m_engine, alignaddr);
	if (0 != res) {
		return res;
	}

	shm_debug("shm_device_takeover offset (%u) OK\n", offset);
	return 0;
}


int shm_device_giveup(shm_device_t *device, uint offset)
{
	int res = 0;
	uint alignaddr = 0;

	shm_debug("shm_device_giveup start (%u)\n", offset);

	if (NULL == device)
		return -EINVAL;

	alignaddr = device->m_engine->m_base + offset;

	res = memory_engine_giveup(device->m_engine,alignaddr);
	if (0 != res) {
		return res;
	}

	shm_debug("shm_device_giveup offset (%u) OK\n", offset);
	return 0;
}

int shm_device_get_stat(shm_device_t *device, struct shm_stat_info *stat)
{
	int res = 0;

	shm_debug("shm_device_get_stat start.\n");

	if (NULL == device || NULL == stat)
		return -EINVAL;

	res = memory_engine_get_stat(device->m_engine, stat);
	if (0 != res) {
		return res;
	}

	shm_debug("shm_device_get_stat OK.\n");
	return 0;
}

int shm_device_release_by_taskid(shm_device_t *device, pid_t taskid)
{
	int res;

	shm_debug("shm_device_release_by_taskid(%d) enter\n",taskid);

	if (device == NULL)
		return -EINVAL;

	res = memory_engine_release_by_taskid(device, taskid);
	if (res != 0) {
		shm_error("memory_engine_release_by_taskid(%d) failed\n", taskid);
		return res;
	}

	shm_debug("shm_device_release_by_taskid(%d) OK\n",taskid);
	return 0;
}

int shm_device_dump_stat(shm_device_t *device)
{
	int res = 0;
	shm_debug("shm_device_dump_stat start.\n");

	if (NULL == device)
		return -EINVAL;

	res = memory_engine_dump_stat(device->m_engine);

	shm_debug("shm_device_dump_stat OK. \n");
	return res;
}

int shm_device_show_stat(shm_device_t *device, struct seq_file *file)
{
	int res = 0;
	shm_debug("shm_device_show_stat start.\n");

	if ((NULL == device) || (NULL == file))
		return -EINVAL;

	/* Walk all nodes and print node info to buffer */
	res = memory_engine_show_stat(device->m_engine, file);

	shm_debug("shm_device_show_stat OK. \n");
	return res;
}
