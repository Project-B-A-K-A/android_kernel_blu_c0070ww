/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM btrfs

#if !defined(_TRACE_BTRFS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BTRFS_H

#include <linux/writeback.h>
#include <linux/tracepoint.h>
#include <trace/events/mmflags.h>

struct btrfs_root;
struct btrfs_fs_info;
struct btrfs_inode;
struct extent_map;
struct btrfs_file_extent_item;
struct btrfs_ordered_extent;
struct btrfs_delayed_ref_node;
struct btrfs_delayed_tree_ref;
struct btrfs_delayed_data_ref;
struct btrfs_delayed_ref_head;
struct btrfs_block_group_cache;
struct btrfs_free_cluster;
struct map_lookup;
struct extent_buffer;
struct btrfs_work;
struct __btrfs_workqueue;
struct btrfs_qgroup_extent_record;
struct btrfs_qgroup;
struct prelim_ref;

#define show_ref_type(type)						\
	__print_symbolic(type,						\
		{ BTRFS_TREE_BLOCK_REF_KEY, 	"TREE_BLOCK_REF" },	\
		{ BTRFS_EXTENT_DATA_REF_KEY, 	"EXTENT_DATA_REF" },	\
		{ BTRFS_EXTENT_REF_V0_KEY, 	"EXTENT_REF_V0" },	\
		{ BTRFS_SHARED_BLOCK_REF_KEY, 	"SHARED_BLOCK_REF" },	\
		{ BTRFS_SHARED_DATA_REF_KEY, 	"SHARED_DATA_REF" })

#define __show_root_type(obj)						\
	__print_symbolic_u64(obj,					\
		{ BTRFS_ROOT_TREE_OBJECTID, 	"ROOT_TREE"	},	\
		{ BTRFS_EXTENT_TREE_OBJECTID, 	"EXTENT_TREE"	},	\
		{ BTRFS_CHUNK_TREE_OBJECTID, 	"CHUNK_TREE"	},	\
		{ BTRFS_DEV_TREE_OBJECTID, 	"DEV_TREE"	},	\
		{ BTRFS_FS_TREE_OBJECTID, 	"FS_TREE"	},	\
		{ BTRFS_ROOT_TREE_DIR_OBJECTID, "ROOT_TREE_DIR"	},	\
		{ BTRFS_CSUM_TREE_OBJECTID, 	"CSUM_TREE"	},	\
		{ BTRFS_TREE_LOG_OBJECTID,	"TREE_LOG"	},	\
		{ BTRFS_QUOTA_TREE_OBJECTID,	"QUOTA_TREE"	},	\
		{ BTRFS_TREE_RELOC_OBJECTID,	"TREE_RELOC"	},	\
		{ BTRFS_UUID_TREE_OBJECTID,	"UUID_TREE"	},	\
		{ BTRFS_FREE_SPACE_TREE_OBJECTID, "FREE_SPACE_TREE" },	\
		{ BTRFS_DATA_RELOC_TREE_OBJECTID, "DATA_RELOC_TREE" })

#define show_root_type(obj)						\
	obj, ((obj >= BTRFS_DATA_RELOC_TREE_OBJECTID) ||		\
	      (obj >= BTRFS_ROOT_TREE_OBJECTID &&			\
	       obj <= BTRFS_QUOTA_TREE_OBJECTID)) ? __show_root_type(obj) : "-"

#define show_fi_type(type)						\
	__print_symbolic(type,						\
		 { BTRFS_FILE_EXTENT_INLINE,	"INLINE" },		\
		 { BTRFS_FILE_EXTENT_REG,	"REG"	 },		\
		 { BTRFS_FILE_EXTENT_PREALLOC,	"PREALLOC"})

#define BTRFS_GROUP_FLAGS	\
	{ BTRFS_BLOCK_GROUP_DATA,	"DATA"},	\
	{ BTRFS_BLOCK_GROUP_SYSTEM,	"SYSTEM"},	\
	{ BTRFS_BLOCK_GROUP_METADATA,	"METADATA"},	\
	{ BTRFS_BLOCK_GROUP_RAID0,	"RAID0"}, 	\
	{ BTRFS_BLOCK_GROUP_RAID1,	"RAID1"}, 	\
	{ BTRFS_BLOCK_GROUP_DUP,	"DUP"}, 	\
	{ BTRFS_BLOCK_GROUP_RAID10,	"RAID10"}, 	\
	{ BTRFS_BLOCK_GROUP_RAID5,	"RAID5"},	\
	{ BTRFS_BLOCK_GROUP_RAID6,	"RAID6"}

#define BTRFS_FSID_SIZE 16
#define TP_STRUCT__entry_fsid __array(u8, fsid, BTRFS_FSID_SIZE)

#define TP_fast_assign_fsid(fs_info)					\
	memcpy(__entry->fsid, fs_info->fsid, BTRFS_FSID_SIZE)

#define TP_STRUCT__entry_btrfs(args...)					\
	TP_STRUCT__entry(						\
		TP_STRUCT__entry_fsid					\
		args)
#define TP_fast_assign_btrfs(fs_info, args...)				\
	TP_fast_assign(							\
		TP_fast_assign_fsid(fs_info);				\
		args)
#define TP_printk_btrfs(fmt, args...) \
	TP_printk("%pU: " fmt, __entry->fsid, args)

TRACE_EVENT(btrfs_transaction_commit,

	TP_PROTO(const struct btrfs_root *root),

	TP_ARGS(root),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  generation		)
		__field(	u64,  root_objectid		)
	),

	TP_fast_assign_btrfs(root->fs_info,
		__entry->generation	= root->fs_info->generation;
		__entry->root_objectid	= root->root_key.objectid;
	),

	TP_printk_btrfs("root = %llu(%s), gen = %llu",
		  show_root_type(__entry->root_objectid),
		  (unsigned long long)__entry->generation)
);

DECLARE_EVENT_CLASS(btrfs__inode,

	TP_PROTO(const struct inode *inode),

	TP_ARGS(inode),

	TP_STRUCT__entry_btrfs(
		__field(	ino_t,  ino			)
		__field(	blkcnt_t,  blocks		)
		__field(	u64,  disk_i_size		)
		__field(	u64,  generation		)
		__field(	u64,  last_trans		)
		__field(	u64,  logged_trans		)
		__field(	u64,  root_objectid		)
	),

	TP_fast_assign_btrfs(btrfs_sb(inode->i_sb),
		__entry->ino	= inode->i_ino;
		__entry->blocks	= inode->i_blocks;
		__entry->disk_i_size  = BTRFS_I(inode)->disk_i_size;
		__entry->generation = BTRFS_I(inode)->generation;
		__entry->last_trans = BTRFS_I(inode)->last_trans;
		__entry->logged_trans = BTRFS_I(inode)->logged_trans;
		__entry->root_objectid =
				BTRFS_I(inode)->root->root_key.objectid;
	),

	TP_printk_btrfs("root=%llu(%s) gen=%llu ino=%lu blocks=%llu "
		  "disk_i_size=%llu last_trans=%llu logged_trans=%llu",
		  show_root_type(__entry->root_objectid),
		  (unsigned long long)__entry->generation,
		  (unsigned long)__entry->ino,
		  (unsigned long long)__entry->blocks,
		  (unsigned long long)__entry->disk_i_size,
		  (unsigned long long)__entry->last_trans,
		  (unsigned long long)__entry->logged_trans)
);

DEFINE_EVENT(btrfs__inode, btrfs_inode_new,

	TP_PROTO(const struct inode *inode),

	TP_ARGS(inode)
);

DEFINE_EVENT(btrfs__inode, btrfs_inode_request,

	TP_PROTO(const struct inode *inode),

	TP_ARGS(inode)
);

DEFINE_EVENT(btrfs__inode, btrfs_inode_evict,

	TP_PROTO(const struct inode *inode),

	TP_ARGS(inode)
);

#define __show_map_type(type)						\
	__print_symbolic_u64(type,					\
		{ EXTENT_MAP_LAST_BYTE, "LAST_BYTE" 	},		\
		{ EXTENT_MAP_HOLE, 	"HOLE" 		},		\
		{ EXTENT_MAP_INLINE, 	"INLINE" 	},		\
		{ EXTENT_MAP_DELALLOC,	"DELALLOC" 	})

#define show_map_type(type)			\
	type, (type >= EXTENT_MAP_LAST_BYTE) ? "-" :  __show_map_type(type)

#define show_map_flags(flag)						\
	__print_flags(flag, "|",					\
		{ (1 << EXTENT_FLAG_PINNED), 		"PINNED" 	},\
		{ (1 << EXTENT_FLAG_COMPRESSED), 	"COMPRESSED" 	},\
		{ (1 << EXTENT_FLAG_VACANCY), 		"VACANCY" 	},\
		{ (1 << EXTENT_FLAG_PREALLOC), 		"PREALLOC" 	},\
		{ (1 << EXTENT_FLAG_LOGGING),	 	"LOGGING" 	},\
		{ (1 << EXTENT_FLAG_FILLING),	 	"FILLING" 	},\
		{ (1 << EXTENT_FLAG_FS_MAPPING),	"FS_MAPPING"	})

TRACE_EVENT_CONDITION(btrfs_get_extent,

	TP_PROTO(const struct btrfs_root *root, const struct btrfs_inode *inode,
		 const struct extent_map *map),

	TP_ARGS(root, inode, map),

	TP_CONDITION(map),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  root_objectid	)
		__field(	u64,  ino		)
		__field(	u64,  start		)
		__field(	u64,  len		)
		__field(	u64,  orig_start	)
		__field(	u64,  block_start	)
		__field(	u64,  block_len		)
		__field(	unsigned long,  flags	)
		__field(	int,  refs		)
		__field(	unsigned int,  compress_type	)
	),

	TP_fast_assign_btrfs(root->fs_info,
		__entry->root_objectid	= root->root_key.objectid;
		__entry->ino		= btrfs_ino(inode);
		__entry->start		= map->start;
		__entry->len		= map->len;
		__entry->orig_start	= map->orig_start;
		__entry->block_start	= map->block_start;
		__entry->block_len	= map->block_len;
		__entry->flags		= map->flags;
		__entry->refs		= refcount_read(&map->refs);
		__entry->compress_type	= map->compress_type;
	),

	TP_printk_btrfs("root=%llu(%s) ino=%llu start=%llu len=%llu "
		  "orig_start=%llu block_start=%llu(%s) "
		  "block_len=%llu flags=%s refs=%u "
		  "compress_type=%u",
		  show_root_type(__entry->root_objectid),
		  (unsigned long long)__entry->ino,
		  (unsigned long long)__entry->start,
		  (unsigned long long)__entry->len,
		  (unsigned long long)__entry->orig_start,
		  show_map_type(__entry->block_start),
		  (unsigned long long)__entry->block_len,
		  show_map_flags(__entry->flags),
		  __entry->refs, __entry->compress_type)
);

/* file extent item */
DECLARE_EVENT_CLASS(btrfs__file_extent_item_regular,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, u64 start),

	TP_ARGS(bi, l, fi, start),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	root_obj	)
		__field(	u64,	ino		)
		__field(	loff_t,	isize		)
		__field(	u64,	disk_isize	)
		__field(	u64,	num_bytes	)
		__field(	u64,	ram_bytes	)
		__field(	u64,	disk_bytenr	)
		__field(	u64,	disk_num_bytes	)
		__field(	u64,	extent_offset	)
		__field(	u8,	extent_type	)
		__field(	u8,	compression	)
		__field(	u64,	extent_start	)
		__field(	u64,	extent_end	)
	),

	TP_fast_assign_btrfs(bi->root->fs_info,
		__entry->root_obj	= bi->root->objectid;
		__entry->ino		= btrfs_ino(bi);
		__entry->isize		= bi->vfs_inode.i_size;
		__entry->disk_isize	= bi->disk_i_size;
		__entry->num_bytes	= btrfs_file_extent_num_bytes(l, fi);
		__entry->ram_bytes	= btrfs_file_extent_ram_bytes(l, fi);
		__entry->disk_bytenr	= btrfs_file_extent_disk_bytenr(l, fi);
		__entry->disk_num_bytes	= btrfs_file_extent_disk_num_bytes(l, fi);
		__entry->extent_offset	= btrfs_file_extent_offset(l, fi);
		__entry->extent_type	= btrfs_file_extent_type(l, fi);
		__entry->compression	= btrfs_file_extent_compression(l, fi);
		__entry->extent_start	= start;
		__entry->extent_end	= (start + __entry->num_bytes);
	),

	TP_printk_btrfs(
		"root=%llu(%s) inode=%llu size=%llu disk_isize=%llu "
		"file extent range=[%llu %llu] "
		"(num_bytes=%llu ram_bytes=%llu disk_bytenr=%llu "
		"disk_num_bytes=%llu extent_offset=%llu type=%s "
		"compression=%u",
		show_root_type(__entry->root_obj), __entry->ino,
		__entry->isize,
		__entry->disk_isize, __entry->extent_start,
		__entry->extent_end, __entry->num_bytes, __entry->ram_bytes,
		__entry->disk_bytenr, __entry->disk_num_bytes,
		__entry->extent_offset, show_fi_type(__entry->extent_type),
		__entry->compression)
);

DECLARE_EVENT_CLASS(
	btrfs__file_extent_item_inline,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, int slot, u64 start),

	TP_ARGS(bi, l, fi, slot,  start),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	root_obj	)
		__field(	u64,	ino		)
		__field(	loff_t,	isize		)
		__field(	u64,	disk_isize	)
		__field(	u8,	extent_type	)
		__field(	u8,	compression	)
		__field(	u64,	extent_start	)
		__field(	u64,	extent_end	)
	),

	TP_fast_assign_btrfs(
		bi->root->fs_info,
		__entry->root_obj	= bi->root->objectid;
		__entry->ino		= btrfs_ino(bi);
		__entry->isize		= bi->vfs_inode.i_size;
		__entry->disk_isize	= bi->disk_i_size;
		__entry->extent_type	= btrfs_file_extent_type(l, fi);
		__entry->compression	= btrfs_file_extent_compression(l, fi);
		__entry->extent_start	= start;
		__entry->extent_end	= (start + btrfs_file_extent_inline_len(l, slot, fi));
	),

	TP_printk_btrfs(
		"root=%llu(%s) inode=%llu size=%llu disk_isize=%llu "
		"file extent range=[%llu %llu] "
		"extent_type=%s compression=%u",
		show_root_type(__entry->root_obj), __entry->ino, __entry->isize,
		__entry->disk_isize, __entry->extent_start,
		__entry->extent_end, show_fi_type(__entry->extent_type),
		__entry->compression)
);

DEFINE_EVENT(
	btrfs__file_extent_item_regular, btrfs_get_extent_show_fi_regular,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, u64 start),

	TP_ARGS(bi, l, fi, start)
);

DEFINE_EVENT(
	btrfs__file_extent_item_regular, btrfs_truncate_show_fi_regular,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, u64 start),

	TP_ARGS(bi, l, fi, start)
);

DEFINE_EVENT(
	btrfs__file_extent_item_inline, btrfs_get_extent_show_fi_inline,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, int slot, u64 start),

	TP_ARGS(bi, l, fi, slot, start)
);

DEFINE_EVENT(
	btrfs__file_extent_item_inline, btrfs_truncate_show_fi_inline,

	TP_PROTO(struct btrfs_inode *bi, struct extent_buffer *l,
		 struct btrfs_file_extent_item *fi, int slot, u64 start),

	TP_ARGS(bi, l, fi, slot, start)
);

#define show_ordered_flags(flags)					   \
	__print_flags(flags, "|",					   \
		{ (1 << BTRFS_ORDERED_IO_DONE), 	"IO_DONE" 	}, \
		{ (1 << BTRFS_ORDERED_COMPLETE), 	"COMPLETE" 	}, \
		{ (1 << BTRFS_ORDERED_NOCOW), 		"NOCOW" 	}, \
		{ (1 << BTRFS_ORDERED_COMPRESSED), 	"COMPRESSED" 	}, \
		{ (1 << BTRFS_ORDERED_PREALLOC), 	"PREALLOC" 	}, \
		{ (1 << BTRFS_ORDERED_DIRECT),	 	"DIRECT" 	}, \
		{ (1 << BTRFS_ORDERED_IOERR), 		"IOERR" 	}, \
		{ (1 << BTRFS_ORDERED_UPDATED_ISIZE), 	"UPDATED_ISIZE"	}, \
		{ (1 << BTRFS_ORDERED_LOGGED_CSUM), 	"LOGGED_CSUM"	}, \
		{ (1 << BTRFS_ORDERED_TRUNCATED), 	"TRUNCATED"	})


DECLARE_EVENT_CLASS(btrfs__ordered_extent,

	TP_PROTO(const struct inode *inode,
		 const struct btrfs_ordered_extent *ordered),

	TP_ARGS(inode, ordered),

	TP_STRUCT__entry_btrfs(
		__field(	ino_t,  ino		)
		__field(	u64,  file_offset	)
		__field(	u64,  start		)
		__field(	u64,  len		)
		__field(	u64,  disk_len		)
		__field(	u64,  bytes_left	)
		__field(	unsigned long,  flags	)
		__field(	int,  compress_type	)
		__field(	int,  refs		)
		__field(	u64,  root_objectid	)
		__field(	u64,  truncated_len	)
	),

	TP_fast_assign_btrfs(btrfs_sb(inode->i_sb),
		__entry->ino 		= inode->i_ino;
		__entry->file_offset	= ordered->file_offset;
		__entry->start		= ordered->start;
		__entry->len		= ordered->len;
		__entry->disk_len	= ordered->disk_len;
		__entry->bytes_left	= ordered->bytes_left;
		__entry->flags		= ordered->flags;
		__entry->compress_type	= ordered->compress_type;
		__entry->refs		= refcount_read(&ordered->refs);
		__entry->root_objectid	=
				BTRFS_I(inode)->root->root_key.objectid;
		__entry->truncated_len	= ordered->truncated_len;
	),

	TP_printk_btrfs("root=%llu(%s) ino=%llu file_offset=%llu "
		  "start=%llu len=%llu disk_len=%llu "
		  "truncated_len=%llu "
		  "bytes_left=%llu flags=%s compress_type=%d "
		  "refs=%d",
		  show_root_type(__entry->root_objectid),
		  (unsigned long long)__entry->ino,
		  (unsigned long long)__entry->file_offset,
		  (unsigned long long)__entry->start,
		  (unsigned long long)__entry->len,
		  (unsigned long long)__entry->disk_len,
		  (unsigned long long)__entry->truncated_len,
		  (unsigned long long)__entry->bytes_left,
		  show_ordered_flags(__entry->flags),
		  __entry->compress_type, __entry->refs)
);

DEFINE_EVENT(btrfs__ordered_extent, btrfs_ordered_extent_add,

	TP_PROTO(const struct inode *inode,
		 const struct btrfs_ordered_extent *ordered),

	TP_ARGS(inode, ordered)
);

DEFINE_EVENT(btrfs__ordered_extent, btrfs_ordered_extent_remove,

	TP_PROTO(const struct inode *inode,
		 const struct btrfs_ordered_extent *ordered),

	TP_ARGS(inode, ordered)
);

DEFINE_EVENT(btrfs__ordered_extent, btrfs_ordered_extent_start,

	TP_PROTO(const struct inode *inode,
		 const struct btrfs_ordered_extent *ordered),

	TP_ARGS(inode, ordered)
);

DEFINE_EVENT(btrfs__ordered_extent, btrfs_ordered_extent_put,

	TP_PROTO(const struct inode *inode,
		 const struct btrfs_ordered_extent *ordered),

	TP_ARGS(inode, ordered)
);

DECLARE_EVENT_CLASS(btrfs__writepage,

	TP_PROTO(const struct page *page, const struct inode *inode,
		 const struct writeback_control *wbc),

	TP_ARGS(page, inode, wbc),

	TP_STRUCT__entry_btrfs(
		__field(	ino_t,  ino			)
		__field(	pgoff_t,  index			)
		__field(	long,   nr_to_write		)
		__field(	long,   pages_skipped		)
		__field(	loff_t, range_start		)
		__field(	loff_t, range_end		)
		__field(	char,   for_kupdate		)
		__field(	char,   for_reclaim		)
		__field(	char,   range_cyclic		)
		__field(	pgoff_t,  writeback_index	)
		__field(	u64,    root_objectid		)
	),

	TP_fast_assign_btrfs(btrfs_sb(inode->i_sb),
		__entry->ino		= inode->i_ino;
		__entry->index		= page->index;
		__entry->nr_to_write	= wbc->nr_to_write;
		__entry->pages_skipped	= wbc->pages_skipped;
		__entry->range_start	= wbc->range_start;
		__entry->range_end	= wbc->range_end;
		__entry->for_kupdate	= wbc->for_kupdate;
		__entry->for_reclaim	= wbc->for_reclaim;
		__entry->range_cyclic	= wbc->range_cyclic;
		__entry->writeback_index = inode->i_mapping->writeback_index;
		__entry->root_objectid	=
				 BTRFS_I(inode)->root->root_key.objectid;
	),

	TP_printk_btrfs("root=%llu(%s) ino=%lu page_index=%lu "
		  "nr_to_write=%ld pages_skipped=%ld range_start=%llu "
		  "range_end=%llu for_kupdate=%d "
		  "for_reclaim=%d range_cyclic=%d writeback_index=%lu",
		  show_root_type(__entry->root_objectid),
		  (unsigned long)__entry->ino, __entry->index,
		  __entry->nr_to_write, __entry->pages_skipped,
		  __entry->range_start, __entry->range_end,
		  __entry->for_kupdate,
		  __entry->for_reclaim, __entry->range_cyclic,
		  (unsigned long)__entry->writeback_index)
);

DEFINE_EVENT(btrfs__writepage, __extent_writepage,

	TP_PROTO(const struct page *page, const struct inode *inode,
		 const struct writeback_control *wbc),

	TP_ARGS(page, inode, wbc)
);

TRACE_EVENT(btrfs_writepage_end_io_hook,

	TP_PROTO(const struct page *page, u64 start, u64 end, int uptodate),

	TP_ARGS(page, start, end, uptodate),

	TP_STRUCT__entry_btrfs(
		__field(	ino_t,	 ino		)
		__field(	pgoff_t, index		)
		__field(	u64,	 start		)
		__field(	u64,	 end		)
		__field(	int,	 uptodate	)
		__field(	u64,    root_objectid	)
	),

	TP_fast_assign_btrfs(btrfs_sb(page->mapping->host->i_sb),
		__entry->ino	= page->mapping->host->i_ino;
		__entry->index	= page->index;
		__entry->start	= start;
		__entry->end	= end;
		__entry->uptodate = uptodate;
		__entry->root_objectid	=
			 BTRFS_I(page->mapping->host)->root->root_key.objectid;
	),

	TP_printk_btrfs("root=%llu(%s) ino=%lu page_index=%lu start=%llu "
		  "end=%llu uptodate=%d",
		  show_root_type(__entry->root_objectid),
		  (unsigned long)__entry->ino, (unsigned long)__entry->index,
		  (unsigned long long)__entry->start,
		  (unsigned long long)__entry->end, __entry->uptodate)
);

TRACE_EVENT(btrfs_sync_file,

	TP_PROTO(const struct file *file, int datasync),

	TP_ARGS(file, datasync),

	TP_STRUCT__entry_btrfs(
		__field(	ino_t,  ino		)
		__field(	ino_t,  parent		)
		__field(	int,    datasync	)
		__field(	u64,    root_objectid	)
	),

	TP_fast_assign(
		const struct dentry *dentry = file->f_path.dentry;
		const struct inode *inode = d_inode(dentry);

		TP_fast_assign_fsid(btrfs_sb(file->f_path.dentry->d_sb));
		__entry->ino		= inode->i_ino;
		__entry->parent		= d_inode(dentry->d_parent)->i_ino;
		__entry->datasync	= datasync;
		__entry->root_objectid	=
				 BTRFS_I(inode)->root->root_key.objectid;
	),

	TP_printk_btrfs("root=%llu(%s) ino=%ld parent=%ld datasync=%d",
		  show_root_type(__entry->root_objectid),
		  (unsigned long)__entry->ino, (unsigned long)__entry->parent,
		  __entry->datasync)
);

TRACE_EVENT(btrfs_sync_fs,

	TP_PROTO(const struct btrfs_fs_info *fs_info, int wait),

	TP_ARGS(fs_info, wait),

	TP_STRUCT__entry_btrfs(
		__field(	int,  wait		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->wait	= wait;
	),

	TP_printk_btrfs("wait = %d", __entry->wait)
);

TRACE_EVENT(btrfs_add_block_group,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_block_group_cache *block_group, int create),

	TP_ARGS(fs_info, block_group, create),

	TP_STRUCT__entry(
		__array(	u8,	fsid,	BTRFS_FSID_SIZE	)
		__field(	u64,	offset			)
		__field(	u64,	size			)
		__field(	u64,	flags			)
		__field(	u64,	bytes_used		)
		__field(	u64,	bytes_super		)
		__field(	int,	create			)
	),

	TP_fast_assign(
		memcpy(__entry->fsid, fs_info->fsid, BTRFS_FSID_SIZE);
		__entry->offset		= block_group->key.objectid;
		__entry->size		= block_group->key.offset;
		__entry->flags		= block_group->flags;
		__entry->bytes_used	=
			btrfs_block_group_used(&block_group->item);
		__entry->bytes_super	= block_group->bytes_super;
		__entry->create		= create;
	),

	TP_printk("%pU: block_group offset=%llu size=%llu "
		  "flags=%llu(%s) bytes_used=%llu bytes_super=%llu "
		  "create=%d", __entry->fsid,
		  (unsigned long long)__entry->offset,
		  (unsigned long long)__entry->size,
		  (unsigned long long)__entry->flags,
		  __print_flags((unsigned long)__entry->flags, "|",
				BTRFS_GROUP_FLAGS),
		  (unsigned long long)__entry->bytes_used,
		  (unsigned long long)__entry->bytes_super, __entry->create)
);

#define show_ref_action(action)						\
	__print_symbolic(action,					\
		{ BTRFS_ADD_DELAYED_REF,    "ADD_DELAYED_REF" },	\
		{ BTRFS_DROP_DELAYED_REF,   "DROP_DELAYED_REF" },	\
		{ BTRFS_ADD_DELAYED_EXTENT, "ADD_DELAYED_EXTENT" }, 	\
		{ BTRFS_UPDATE_DELAYED_HEAD, "UPDATE_DELAYED_HEAD" })
			

DECLARE_EVENT_CLASS(btrfs_delayed_tree_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_tree_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  bytenr		)
		__field(	u64,  num_bytes		)
		__field(	int,  action		) 
		__field(	u64,  parent		)
		__field(	u64,  ref_root		)
		__field(	int,  level		)
		__field(	int,  type		)
		__field(	u64,  seq		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bytenr		= ref->bytenr;
		__entry->num_bytes	= ref->num_bytes;
		__entry->action		= action;
		__entry->parent		= full_ref->parent;
		__entry->ref_root	= full_ref->root;
		__entry->level		= full_ref->level;
		__entry->type		= ref->type;
		__entry->seq		= ref->seq;
	),

	TP_printk_btrfs("bytenr=%llu num_bytes=%llu action=%s "
		  "parent=%llu(%s) ref_root=%llu(%s) level=%d "
		  "type=%s seq=%llu",
		  (unsigned long long)__entry->bytenr,
		  (unsigned long long)__entry->num_bytes,
		  show_ref_action(__entry->action),
		  show_root_type(__entry->parent),
		  show_root_type(__entry->ref_root),
		  __entry->level, show_ref_type(__entry->type),
		  (unsigned long long)__entry->seq)
);

DEFINE_EVENT(btrfs_delayed_tree_ref,  add_delayed_tree_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_tree_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action)
);

DEFINE_EVENT(btrfs_delayed_tree_ref,  run_delayed_tree_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_tree_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action)
);

DECLARE_EVENT_CLASS(btrfs_delayed_data_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_data_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  bytenr		)
		__field(	u64,  num_bytes		)
		__field(	int,  action		) 
		__field(	u64,  parent		)
		__field(	u64,  ref_root		)
		__field(	u64,  owner		)
		__field(	u64,  offset		)
		__field(	int,  type		)
		__field(	u64,  seq		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bytenr		= ref->bytenr;
		__entry->num_bytes	= ref->num_bytes;
		__entry->action		= action;
		__entry->parent		= full_ref->parent;
		__entry->ref_root	= full_ref->root;
		__entry->owner		= full_ref->objectid;
		__entry->offset		= full_ref->offset;
		__entry->type		= ref->type;
		__entry->seq		= ref->seq;
	),

	TP_printk_btrfs("bytenr=%llu num_bytes=%llu action=%s "
		  "parent=%llu(%s) ref_root=%llu(%s) owner=%llu "
		  "offset=%llu type=%s seq=%llu",
		  (unsigned long long)__entry->bytenr,
		  (unsigned long long)__entry->num_bytes,
		  show_ref_action(__entry->action),
		  show_root_type(__entry->parent),
		  show_root_type(__entry->ref_root),
		  (unsigned long long)__entry->owner,
		  (unsigned long long)__entry->offset,
		  show_ref_type(__entry->type),
		  (unsigned long long)__entry->seq)
);

DEFINE_EVENT(btrfs_delayed_data_ref,  add_delayed_data_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_data_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action)
);

DEFINE_EVENT(btrfs_delayed_data_ref,  run_delayed_data_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_data_ref *full_ref,
		 int action),

	TP_ARGS(fs_info, ref, full_ref, action)
);

DECLARE_EVENT_CLASS(btrfs_delayed_ref_head,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_ref_head *head_ref,
		 int action),

	TP_ARGS(fs_info, ref, head_ref, action),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  bytenr		)
		__field(	u64,  num_bytes		)
		__field(	int,  action		) 
		__field(	int,  is_data		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bytenr		= ref->bytenr;
		__entry->num_bytes	= ref->num_bytes;
		__entry->action		= action;
		__entry->is_data	= head_ref->is_data;
	),

	TP_printk_btrfs("bytenr=%llu num_bytes=%llu action=%s is_data=%d",
		  (unsigned long long)__entry->bytenr,
		  (unsigned long long)__entry->num_bytes,
		  show_ref_action(__entry->action),
		  __entry->is_data)
);

DEFINE_EVENT(btrfs_delayed_ref_head,  add_delayed_ref_head,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_ref_head *head_ref,
		 int action),

	TP_ARGS(fs_info, ref, head_ref, action)
);

DEFINE_EVENT(btrfs_delayed_ref_head,  run_delayed_ref_head,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_delayed_ref_node *ref,
		 const struct btrfs_delayed_ref_head *head_ref,
		 int action),

	TP_ARGS(fs_info, ref, head_ref, action)
);

#define show_chunk_type(type)					\
	__print_flags(type, "|",				\
		{ BTRFS_BLOCK_GROUP_DATA, 	"DATA"	},	\
		{ BTRFS_BLOCK_GROUP_SYSTEM, 	"SYSTEM"},	\
		{ BTRFS_BLOCK_GROUP_METADATA, 	"METADATA"},	\
		{ BTRFS_BLOCK_GROUP_RAID0, 	"RAID0" },	\
		{ BTRFS_BLOCK_GROUP_RAID1, 	"RAID1" },	\
		{ BTRFS_BLOCK_GROUP_DUP, 	"DUP"	},	\
		{ BTRFS_BLOCK_GROUP_RAID10, 	"RAID10"},	\
		{ BTRFS_BLOCK_GROUP_RAID5, 	"RAID5"	},	\
		{ BTRFS_BLOCK_GROUP_RAID6, 	"RAID6"	})

DECLARE_EVENT_CLASS(btrfs__chunk,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct map_lookup *map, u64 offset, u64 size),

	TP_ARGS(fs_info, map, offset, size),

	TP_STRUCT__entry_btrfs(
		__field(	int,  num_stripes		)
		__field(	u64,  type			)
		__field(	int,  sub_stripes		)
		__field(	u64,  offset			)
		__field(	u64,  size			)
		__field(	u64,  root_objectid		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->num_stripes	= map->num_stripes;
		__entry->type		= map->type;
		__entry->sub_stripes	= map->sub_stripes;
		__entry->offset		= offset;
		__entry->size		= size;
		__entry->root_objectid	= fs_info->chunk_root->root_key.objectid;
	),

	TP_printk_btrfs("root=%llu(%s) offset=%llu size=%llu "
		  "num_stripes=%d sub_stripes=%d type=%s",
		  show_root_type(__entry->root_objectid),
		  (unsigned long long)__entry->offset,
		  (unsigned long long)__entry->size,
		  __entry->num_stripes, __entry->sub_stripes,
		  show_chunk_type(__entry->type))
);

DEFINE_EVENT(btrfs__chunk,  btrfs_chunk_alloc,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct map_lookup *map, u64 offset, u64 size),

	TP_ARGS(fs_info, map, offset, size)
);

DEFINE_EVENT(btrfs__chunk,  btrfs_chunk_free,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct map_lookup *map, u64 offset, u64 size),

	TP_ARGS(fs_info, map, offset, size)
);

TRACE_EVENT(btrfs_cow_block,

	TP_PROTO(const struct btrfs_root *root, const struct extent_buffer *buf,
		 const struct extent_buffer *cow),

	TP_ARGS(root, buf, cow),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  root_objectid		)
		__field(	u64,  buf_start			)
		__field(	int,  refs			)
		__field(	u64,  cow_start			)
		__field(	int,  buf_level			)
		__field(	int,  cow_level			)
	),

	TP_fast_assign_btrfs(root->fs_info,
		__entry->root_objectid	= root->root_key.objectid;
		__entry->buf_start	= buf->start;
		__entry->refs		= atomic_read(&buf->refs);
		__entry->cow_start	= cow->start;
		__entry->buf_level	= btrfs_header_level(buf);
		__entry->cow_level	= btrfs_header_level(cow);
	),

	TP_printk_btrfs("root=%llu(%s) refs=%d orig_buf=%llu "
		  "(orig_level=%d) cow_buf=%llu (cow_level=%d)",
		  show_root_type(__entry->root_objectid),
		  __entry->refs,
		  (unsigned long long)__entry->buf_start,
		  __entry->buf_level,
		  (unsigned long long)__entry->cow_start,
		  __entry->cow_level)
);

TRACE_EVENT(btrfs_space_reservation,

	TP_PROTO(const struct btrfs_fs_info *fs_info, char *type, u64 val,
		 u64 bytes, int reserve),

	TP_ARGS(fs_info, type, val, bytes, reserve),

	TP_STRUCT__entry_btrfs(
		__string(	type,	type			)
		__field(	u64,	val			)
		__field(	u64,	bytes			)
		__field(	int,	reserve			)
	),

	TP_fast_assign_btrfs(fs_info,
		__assign_str(type, type);
		__entry->val		= val;
		__entry->bytes		= bytes;
		__entry->reserve	= reserve;
	),

	TP_printk_btrfs("%s: %Lu %s %Lu", __get_str(type), __entry->val,
			__entry->reserve ? "reserve" : "release",
			__entry->bytes)
);

#define show_flush_action(action)						\
	__print_symbolic(action,						\
		{ BTRFS_RESERVE_NO_FLUSH,	"BTRFS_RESERVE_NO_FLUSH"},	\
		{ BTRFS_RESERVE_FLUSH_LIMIT,	"BTRFS_RESERVE_FLUSH_LIMIT"},	\
		{ BTRFS_RESERVE_FLUSH_ALL,	"BTRFS_RESERVE_FLUSH_ALL"})

TRACE_EVENT(btrfs_trigger_flush,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 flags, u64 bytes,
		 int flush, char *reason),

	TP_ARGS(fs_info, flags, bytes, flush, reason),

	TP_STRUCT__entry(
		__array(	u8,	fsid,	BTRFS_FSID_SIZE	)
		__field(	u64,	flags			)
		__field(	u64,	bytes			)
		__field(	int,	flush			)
		__string(	reason,	reason			)
	),

	TP_fast_assign(
		memcpy(__entry->fsid, fs_info->fsid, BTRFS_FSID_SIZE);
		__entry->flags	= flags;
		__entry->bytes	= bytes;
		__entry->flush	= flush;
		__assign_str(reason, reason)
	),

	TP_printk("%pU: %s: flush=%d(%s) flags=%llu(%s) bytes=%llu",
		  __entry->fsid, __get_str(reason), __entry->flush,
		  show_flush_action(__entry->flush),
		  (unsigned long long)__entry->flags,
		  __print_flags((unsigned long)__entry->flags, "|",
				BTRFS_GROUP_FLAGS),
		  (unsigned long long)__entry->bytes)
);

#define show_flush_state(state)							\
	__print_symbolic(state,							\
		{ FLUSH_DELAYED_ITEMS_NR,	"FLUSH_DELAYED_ITEMS_NR"},	\
		{ FLUSH_DELAYED_ITEMS,		"FLUSH_DELAYED_ITEMS"},		\
		{ FLUSH_DELALLOC,		"FLUSH_DELALLOC"},		\
		{ FLUSH_DELALLOC_WAIT,		"FLUSH_DELALLOC_WAIT"},		\
		{ ALLOC_CHUNK,			"ALLOC_CHUNK"},			\
		{ COMMIT_TRANS,			"COMMIT_TRANS"})

TRACE_EVENT(btrfs_flush_space,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 flags, u64 num_bytes,
		 int state, int ret),

	TP_ARGS(fs_info, flags, num_bytes, state, ret),

	TP_STRUCT__entry(
		__array(	u8,	fsid,	BTRFS_FSID_SIZE	)
		__field(	u64,	flags			)
		__field(	u64,	num_bytes		)
		__field(	int,	state			)
		__field(	int,	ret			)
	),

	TP_fast_assign(
		memcpy(__entry->fsid, fs_info->fsid, BTRFS_FSID_SIZE);
		__entry->flags		=	flags;
		__entry->num_bytes	=	num_bytes;
		__entry->state		=	state;
		__entry->ret		=	ret;
	),

	TP_printk("%pU: state=%d(%s) flags=%llu(%s) num_bytes=%llu ret=%d",
		  __entry->fsid, __entry->state,
		  show_flush_state(__entry->state),
		  (unsigned long long)__entry->flags,
		  __print_flags((unsigned long)__entry->flags, "|",
				BTRFS_GROUP_FLAGS),
		  (unsigned long long)__entry->num_bytes, __entry->ret)
);

DECLARE_EVENT_CLASS(btrfs__reserved_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 start, u64 len),

	TP_ARGS(fs_info, start, len),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  start			)
		__field(	u64,  len			)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->start		= start;
		__entry->len		= len;
	),

	TP_printk_btrfs("root=%llu(%s) start=%llu len=%llu",
		  show_root_type(BTRFS_EXTENT_TREE_OBJECTID),
		  (unsigned long long)__entry->start,
		  (unsigned long long)__entry->len)
);

DEFINE_EVENT(btrfs__reserved_extent,  btrfs_reserved_extent_alloc,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 start, u64 len),

	TP_ARGS(fs_info, start, len)
);

DEFINE_EVENT(btrfs__reserved_extent,  btrfs_reserved_extent_free,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 start, u64 len),

	TP_ARGS(fs_info, start, len)
);

TRACE_EVENT(find_free_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 num_bytes,
		 u64 empty_size, u64 data),

	TP_ARGS(fs_info, num_bytes, empty_size, data),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	num_bytes		)
		__field(	u64,	empty_size		)
		__field(	u64,	data			)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->num_bytes	= num_bytes;
		__entry->empty_size	= empty_size;
		__entry->data		= data;
	),

	TP_printk_btrfs("root=%Lu(%s) len=%Lu empty_size=%Lu flags=%Lu(%s)",
		  show_root_type(BTRFS_EXTENT_TREE_OBJECTID),
		  __entry->num_bytes, __entry->empty_size, __entry->data,
		  __print_flags((unsigned long)__entry->data, "|",
				 BTRFS_GROUP_FLAGS))
);

DECLARE_EVENT_CLASS(btrfs__reserve_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_block_group_cache *block_group, u64 start,
		 u64 len),

	TP_ARGS(fs_info, block_group, start, len),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	bg_objectid		)
		__field(	u64,	flags			)
		__field(	u64,	start			)
		__field(	u64,	len			)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bg_objectid	= block_group->key.objectid;
		__entry->flags		= block_group->flags;
		__entry->start		= start;
		__entry->len		= len;
	),

	TP_printk_btrfs("root=%Lu(%s) block_group=%Lu flags=%Lu(%s) "
		  "start=%Lu len=%Lu",
		  show_root_type(BTRFS_EXTENT_TREE_OBJECTID),
		  __entry->bg_objectid,
		  __entry->flags, __print_flags((unsigned long)__entry->flags,
						"|", BTRFS_GROUP_FLAGS),
		  __entry->start, __entry->len)
);

DEFINE_EVENT(btrfs__reserve_extent, btrfs_reserve_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_block_group_cache *block_group, u64 start,
		 u64 len),

	TP_ARGS(fs_info, block_group, start, len)
);

DEFINE_EVENT(btrfs__reserve_extent, btrfs_reserve_extent_cluster,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_block_group_cache *block_group, u64 start,
		 u64 len),

	TP_ARGS(fs_info, block_group, start, len)
);

TRACE_EVENT(btrfs_find_cluster,

	TP_PROTO(const struct btrfs_block_group_cache *block_group, u64 start,
		 u64 bytes, u64 empty_size, u64 min_bytes),

	TP_ARGS(block_group, start, bytes, empty_size, min_bytes),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	bg_objectid		)
		__field(	u64,	flags			)
		__field(	u64,	start			)
		__field(	u64,	bytes			)
		__field(	u64,	empty_size		)
		__field(	u64,	min_bytes		)
	),

	TP_fast_assign_btrfs(block_group->fs_info,
		__entry->bg_objectid	= block_group->key.objectid;
		__entry->flags		= block_group->flags;
		__entry->start		= start;
		__entry->bytes		= bytes;
		__entry->empty_size	= empty_size;
		__entry->min_bytes	= min_bytes;
	),

	TP_printk_btrfs("block_group=%Lu flags=%Lu(%s) start=%Lu len=%Lu "
		  "empty_size=%Lu min_bytes=%Lu", __entry->bg_objectid,
		  __entry->flags,
		  __print_flags((unsigned long)__entry->flags, "|",
				BTRFS_GROUP_FLAGS), __entry->start,
		  __entry->bytes, __entry->empty_size,  __entry->min_bytes)
);

TRACE_EVENT(btrfs_failed_cluster_setup,

	TP_PROTO(const struct btrfs_block_group_cache *block_group),

	TP_ARGS(block_group),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	bg_objectid		)
	),

	TP_fast_assign_btrfs(block_group->fs_info,
		__entry->bg_objectid	= block_group->key.objectid;
	),

	TP_printk_btrfs("block_group=%Lu", __entry->bg_objectid)
);

TRACE_EVENT(btrfs_setup_cluster,

	TP_PROTO(const struct btrfs_block_group_cache *block_group,
		 const struct btrfs_free_cluster *cluster,
		 u64 size, int bitmap),

	TP_ARGS(block_group, cluster, size, bitmap),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	bg_objectid		)
		__field(	u64,	flags			)
		__field(	u64,	start			)
		__field(	u64,	max_size		)
		__field(	u64,	size			)
		__field(	int,	bitmap			)
	),

	TP_fast_assign_btrfs(block_group->fs_info,
		__entry->bg_objectid	= block_group->key.objectid;
		__entry->flags		= block_group->flags;
		__entry->start		= cluster->window_start;
		__entry->max_size	= cluster->max_size;
		__entry->size		= size;
		__entry->bitmap		= bitmap;
	),

	TP_printk_btrfs("block_group=%Lu flags=%Lu(%s) window_start=%Lu "
		  "size=%Lu max_size=%Lu bitmap=%d",
		  __entry->bg_objectid,
		  __entry->flags,
		  __print_flags((unsigned long)__entry->flags, "|",
				BTRFS_GROUP_FLAGS), __entry->start,
		  __entry->size, __entry->max_size, __entry->bitmap)
);

struct extent_state;
TRACE_EVENT(alloc_extent_state,

	TP_PROTO(const struct extent_state *state,
		 gfp_t mask, unsigned long IP),

	TP_ARGS(state, mask, IP),

	TP_STRUCT__entry(
		__field(const struct extent_state *, state)
		__field(gfp_t, mask)
		__field(unsigned long, ip)
	),

	TP_fast_assign(
		__entry->state	= state,
		__entry->mask	= mask,
		__entry->ip	= IP
	),

	TP_printk("state=%p mask=%s caller=%pS", __entry->state,
		  show_gfp_flags(__entry->mask), (const void *)__entry->ip)
);

TRACE_EVENT(free_extent_state,

	TP_PROTO(const struct extent_state *state, unsigned long IP),

	TP_ARGS(state, IP),

	TP_STRUCT__entry(
		__field(const struct extent_state *, state)
		__field(unsigned long, ip)
	),

	TP_fast_assign(
		__entry->state	= state,
		__entry->ip = IP
	),

	TP_printk("state=%p caller=%pS", __entry->state,
		  (const void *)__entry->ip)
);

DECLARE_EVENT_CLASS(btrfs__work,

	TP_PROTO(const struct btrfs_work *work),

	TP_ARGS(work),

	TP_STRUCT__entry_btrfs(
		__field(	const void *,	work			)
		__field(	const void *,	wq			)
		__field(	const void *,	func			)
		__field(	const void *,	ordered_func		)
		__field(	const void *,	ordered_free		)
		__field(	const void *,	normal_work		)
	),

	TP_fast_assign_btrfs(btrfs_work_owner(work),
		__entry->work		= work;
		__entry->wq		= work->wq;
		__entry->func		= work->func;
		__entry->ordered_func	= work->ordered_func;
		__entry->ordered_free	= work->ordered_free;
		__entry->normal_work	= &work->normal_work;
	),

	TP_printk_btrfs("work=%p (normal_work=%p) wq=%p func=%pf ordered_func=%p "
		  "ordered_free=%p",
		  __entry->work, __entry->normal_work, __entry->wq,
		   __entry->func, __entry->ordered_func, __entry->ordered_free)
);

/*
 * For situiations when the work is freed, we pass fs_info and a tag that that
 * matches address of the work structure so it can be paired with the
 * scheduling event.
 */
DECLARE_EVENT_CLASS(btrfs__work__done,

	TP_PROTO(const struct btrfs_fs_info *fs_info, const void *wtag),

	TP_ARGS(fs_info, wtag),

	TP_STRUCT__entry_btrfs(
		__field(	const void *,	wtag			)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->wtag		= wtag;
	),

	TP_printk_btrfs("work->%p", __entry->wtag)
);

DEFINE_EVENT(btrfs__work, btrfs_work_queued,

	TP_PROTO(const struct btrfs_work *work),

	TP_ARGS(work)
);

DEFINE_EVENT(btrfs__work, btrfs_work_sched,

	TP_PROTO(const struct btrfs_work *work),

	TP_ARGS(work)
);

DEFINE_EVENT(btrfs__work__done, btrfs_all_work_done,

	TP_PROTO(const struct btrfs_fs_info *fs_info, const void *wtag),

	TP_ARGS(fs_info, wtag)
);

DEFINE_EVENT(btrfs__work, btrfs_ordered_sched,

	TP_PROTO(const struct btrfs_work *work),

	TP_ARGS(work)
);

DECLARE_EVENT_CLASS(btrfs__workqueue,

	TP_PROTO(const struct __btrfs_workqueue *wq,
		 const char *name, int high),

	TP_ARGS(wq, name, high),

	TP_STRUCT__entry_btrfs(
		__field(	const void *,	wq			)
		__string(	name,	name			)
		__field(	int ,	high			)
	),

	TP_fast_assign_btrfs(btrfs_workqueue_owner(wq),
		__entry->wq		= wq;
		__assign_str(name, name);
		__entry->high		= high;
	),

	TP_printk_btrfs("name=%s%s wq=%p", __get_str(name),
		  __print_flags(__entry->high, "",
				{(WQ_HIGHPRI),	"-high"}),
		  __entry->wq)
);

DEFINE_EVENT(btrfs__workqueue, btrfs_workqueue_alloc,

	TP_PROTO(const struct __btrfs_workqueue *wq,
		 const char *name, int high),

	TP_ARGS(wq, name, high)
);

DECLARE_EVENT_CLASS(btrfs__workqueue_done,

	TP_PROTO(const struct __btrfs_workqueue *wq),

	TP_ARGS(wq),

	TP_STRUCT__entry_btrfs(
		__field(	const void *,	wq		)
	),

	TP_fast_assign_btrfs(btrfs_workqueue_owner(wq),
		__entry->wq		= wq;
	),

	TP_printk_btrfs("wq=%p", __entry->wq)
);

DEFINE_EVENT(btrfs__workqueue_done, btrfs_workqueue_destroy,

	TP_PROTO(const struct __btrfs_workqueue *wq),

	TP_ARGS(wq)
);

#define BTRFS_QGROUP_OPERATIONS				\
	{ QGROUP_RESERVE,	"reserve"	},	\
	{ QGROUP_RELEASE,	"release"	},	\
	{ QGROUP_FREE,		"free"		}

DECLARE_EVENT_CLASS(btrfs__qgroup_rsv_data,

	TP_PROTO(const struct inode *inode, u64 start, u64 len,
		 u64 reserved, int op),

	TP_ARGS(inode, start, len, reserved, op),

	TP_STRUCT__entry_btrfs(
		__field(	u64,		rootid		)
		__field(	unsigned long,	ino		)
		__field(	u64,		start		)
		__field(	u64,		len		)
		__field(	u64,		reserved	)
		__field(	int,		op		)
	),

	TP_fast_assign_btrfs(btrfs_sb(inode->i_sb),
		__entry->rootid		= BTRFS_I(inode)->root->objectid;
		__entry->ino		= inode->i_ino;
		__entry->start		= start;
		__entry->len		= len;
		__entry->reserved	= reserved;
		__entry->op		= op;
	),

	TP_printk_btrfs("root=%llu ino=%lu start=%llu len=%llu reserved=%llu op=%s",
		  __entry->rootid, __entry->ino, __entry->start, __entry->len,
		  __entry->reserved,
		  __print_flags((unsigned long)__entry->op, "",
				BTRFS_QGROUP_OPERATIONS)
	)
);

DEFINE_EVENT(btrfs__qgroup_rsv_data, btrfs_qgroup_reserve_data,

	TP_PROTO(const struct inode *inode, u64 start, u64 len,
		 u64 reserved, int op),

	TP_ARGS(inode, start, len, reserved, op)
);

DEFINE_EVENT(btrfs__qgroup_rsv_data, btrfs_qgroup_release_data,

	TP_PROTO(const struct inode *inode, u64 start, u64 len,
		 u64 reserved, int op),

	TP_ARGS(inode, start, len, reserved, op)
);

DECLARE_EVENT_CLASS(btrfs__qgroup_delayed_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 u64 ref_root, u64 reserved),

	TP_ARGS(fs_info, ref_root, reserved),

	TP_STRUCT__entry_btrfs(
		__field(	u64,		ref_root	)
		__field(	u64,		reserved	)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->ref_root	= ref_root;
		__entry->reserved	= reserved;
	),

	TP_printk_btrfs("root=%llu reserved=%llu op=free",
		  __entry->ref_root, __entry->reserved)
);

DEFINE_EVENT(btrfs__qgroup_delayed_ref, btrfs_qgroup_free_delayed_ref,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 u64 ref_root, u64 reserved),

	TP_ARGS(fs_info, ref_root, reserved)
);

DECLARE_EVENT_CLASS(btrfs_qgroup_extent,
	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_qgroup_extent_record *rec),

	TP_ARGS(fs_info, rec),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  bytenr		)
		__field(	u64,  num_bytes		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bytenr		= rec->bytenr,
		__entry->num_bytes	= rec->num_bytes;
	),

	TP_printk_btrfs("bytenr=%llu num_bytes=%llu",
		  (unsigned long long)__entry->bytenr,
		  (unsigned long long)__entry->num_bytes)
);

DEFINE_EVENT(btrfs_qgroup_extent, btrfs_qgroup_account_extents,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_qgroup_extent_record *rec),

	TP_ARGS(fs_info, rec)
);

DEFINE_EVENT(btrfs_qgroup_extent, btrfs_qgroup_trace_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct btrfs_qgroup_extent_record *rec),

	TP_ARGS(fs_info, rec)
);

TRACE_EVENT(btrfs_qgroup_account_extent,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 bytenr,
		 u64 num_bytes, u64 nr_old_roots, u64 nr_new_roots),

	TP_ARGS(fs_info, bytenr, num_bytes, nr_old_roots, nr_new_roots),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  bytenr			)
		__field(	u64,  num_bytes			)
		__field(	u64,  nr_old_roots		)
		__field(	u64,  nr_new_roots		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->bytenr		= bytenr;
		__entry->num_bytes	= num_bytes;
		__entry->nr_old_roots	= nr_old_roots;
		__entry->nr_new_roots	= nr_new_roots;
	),

	TP_printk_btrfs("bytenr=%llu num_bytes=%llu nr_old_roots=%llu "
		  "nr_new_roots=%llu",
		  __entry->bytenr,
		  __entry->num_bytes,
		  __entry->nr_old_roots,
		  __entry->nr_new_roots)
);

TRACE_EVENT(qgroup_update_counters,

	TP_PROTO(const struct btrfs_fs_info *fs_info, u64 qgid,
		 u64 cur_old_count, u64 cur_new_count),

	TP_ARGS(fs_info, qgid, cur_old_count, cur_new_count),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  qgid			)
		__field(	u64,  cur_old_count		)
		__field(	u64,  cur_new_count		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->qgid		= qgid;
		__entry->cur_old_count	= cur_old_count;
		__entry->cur_new_count	= cur_new_count;
	),

	TP_printk_btrfs("qgid=%llu cur_old_count=%llu cur_new_count=%llu",
		  __entry->qgid,
		  __entry->cur_old_count,
		  __entry->cur_new_count)
);

TRACE_EVENT(qgroup_update_reserve,

	TP_PROTO(struct btrfs_fs_info *fs_info, struct btrfs_qgroup *qgroup,
		 s64 diff),

	TP_ARGS(fs_info, qgroup, diff),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	qgid			)
		__field(	u64,	cur_reserved		)
		__field(	s64,	diff			)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->qgid		= qgroup->qgroupid;
		__entry->cur_reserved	= qgroup->reserved;
		__entry->diff		= diff;
	),

	TP_printk_btrfs("qgid=%llu cur_reserved=%llu diff=%lld",
		__entry->qgid, __entry->cur_reserved, __entry->diff)
);

TRACE_EVENT(qgroup_meta_reserve,

	TP_PROTO(struct btrfs_root *root, s64 diff),

	TP_ARGS(root, diff),

	TP_STRUCT__entry_btrfs(
		__field(	u64,	refroot			)
		__field(	s64,	diff			)
	),

	TP_fast_assign_btrfs(root->fs_info,
		__entry->refroot	= root->objectid;
		__entry->diff		= diff;
	),

	TP_printk_btrfs("refroot=%llu(%s) diff=%lld",
		show_root_type(__entry->refroot), __entry->diff)
);

DECLARE_EVENT_CLASS(btrfs__prelim_ref,
	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct prelim_ref *oldref,
		 const struct prelim_ref *newref, u64 tree_size),
	TP_ARGS(fs_info, newref, oldref, tree_size),

	TP_STRUCT__entry_btrfs(
		__field(	u64,  root_id		)
		__field(	u64,  objectid		)
		__field(	 u8,  type		)
		__field(	u64,  offset		)
		__field(	int,  level		)
		__field(	int,  old_count		)
		__field(	u64,  parent		)
		__field(	u64,  bytenr		)
		__field(	int,  mod_count		)
		__field(	u64,  tree_size		)
	),

	TP_fast_assign_btrfs(fs_info,
		__entry->root_id	= oldref->root_id;
		__entry->objectid	= oldref->key_for_search.objectid;
		__entry->type		= oldref->key_for_search.type;
		__entry->offset		= oldref->key_for_search.offset;
		__entry->level		= oldref->level;
		__entry->old_count	= oldref->count;
		__entry->parent		= oldref->parent;
		__entry->bytenr		= oldref->wanted_disk_byte;
		__entry->mod_count	= newref ? newref->count : 0;
		__entry->tree_size	= tree_size;
	),

	TP_printk_btrfs("root_id=%llu key=[%llu,%u,%llu] level=%d count=[%d+%d=%d] parent=%llu wanted_disk_byte=%llu nodes=%llu",
			(unsigned long long)__entry->root_id,
			(unsigned long long)__entry->objectid, __entry->type,
			(unsigned long long)__entry->offset, __entry->level,
			__entry->old_count, __entry->mod_count,
			__entry->old_count + __entry->mod_count,
			(unsigned long long)__entry->parent,
			(unsigned long long)__entry->bytenr,
			(unsigned long long)__entry->tree_size)
);

DEFINE_EVENT(btrfs__prelim_ref, btrfs_prelim_ref_merge,
	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct prelim_ref *oldref,
		 const struct prelim_ref *newref, u64 tree_size),
	TP_ARGS(fs_info, oldref, newref, tree_size)
);

DEFINE_EVENT(btrfs__prelim_ref, btrfs_prelim_ref_insert,
	TP_PROTO(const struct btrfs_fs_info *fs_info,
		 const struct prelim_ref *oldref,
		 const struct prelim_ref *newref, u64 tree_size),
	TP_ARGS(fs_info, oldref, newref, tree_size)
);

#endif /* _TRACE_BTRFS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>