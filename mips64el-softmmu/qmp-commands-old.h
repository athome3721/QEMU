

{
.name       = "quit",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_quit,
},


{
.name       = "eject",
.args_type  = "force:-f,device:B",
.mhandler.cmd_new = qmp_marshal_input_eject,
},


{
.name       = "change",
.args_type  = "device:B,target:F,arg:s?",
.mhandler.cmd_new = qmp_marshal_input_change,
},


{
.name       = "screendump",
.args_type  = "filename:F",
.mhandler.cmd_new = qmp_marshal_input_screendump,
},


{
.name       = "stop",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_stop,
},


{
.name       = "cont",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_cont,
},


{
.name       = "system_wakeup",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_system_wakeup,
},


{
.name       = "system_reset",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_system_reset,
},


{
.name       = "system_powerdown",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_system_powerdown,
},


{
.name       = "device_add",
.args_type  = "device:O",
.params     = "driver[,prop=value][,...]",
.help       = "add device, like -device on the command line",
.user_print = monitor_user_noop,
.mhandler.cmd_new = do_device_add,
},


{
.name       = "device_del",
.args_type  = "id:s",
.mhandler.cmd_new = qmp_marshal_input_device_del,
},


{
.name       = "send-key",
.args_type  = "keys:O,hold-time:i?",
.mhandler.cmd_new = qmp_marshal_input_send_key,
},


{
.name       = "cpu",
.args_type  = "index:i",
.mhandler.cmd_new = qmp_marshal_input_cpu,
},


{
.name       = "memsave",
.args_type  = "val:l,size:i,filename:s,cpu:i?",
.mhandler.cmd_new = qmp_marshal_input_memsave,
},


{
.name       = "pmemsave",
.args_type  = "val:l,size:i,filename:s",
.mhandler.cmd_new = qmp_marshal_input_pmemsave,
},


{
.name       = "inject-nmi",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_inject_nmi,
},


{
.name       = "ringbuf-write",
.args_type  = "device:s,data:s,format:s?",
.mhandler.cmd_new = qmp_marshal_input_ringbuf_write,
},


{
.name       = "ringbuf-read",
.args_type  = "device:s,size:i,format:s?",
.mhandler.cmd_new = qmp_marshal_input_ringbuf_read,
},


{
.name       = "xen-save-devices-state",
.args_type  = "filename:F",
.mhandler.cmd_new = qmp_marshal_input_xen_save_devices_state,
},


{
.name       = "xen-set-global-dirty-log",
.args_type  = "enable:b",
.mhandler.cmd_new = qmp_marshal_input_xen_set_global_dirty_log,
},


{
.name       = "migrate",
.args_type  = "detach:-d,blk:-b,inc:-i,uri:s",
.mhandler.cmd_new = qmp_marshal_input_migrate,
},


{
.name       = "migrate_cancel",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_migrate_cancel,
},

{
.name       = "migrate-set-cache-size",
.args_type  = "value:o",
.mhandler.cmd_new = qmp_marshal_input_migrate_set_cache_size,
},

{
.name       = "query-migrate-cache-size",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_migrate_cache_size,
},


{
.name       = "migrate_set_speed",
.args_type  = "value:o",
.mhandler.cmd_new = qmp_marshal_input_migrate_set_speed,
},


{
.name       = "migrate_set_downtime",
.args_type  = "value:T",
.mhandler.cmd_new = qmp_marshal_input_migrate_set_downtime,
},


{
.name       = "client_migrate_info",
.args_type  = "protocol:s,hostname:s,port:i?,tls-port:i?,cert-subject:s?",
.params     = "protocol hostname port tls-port cert-subject",
.help       = "send migration info to spice/vnc client",
.user_print = monitor_user_noop,
.mhandler.cmd_async = client_migrate_info,
.flags      = MONITOR_CMD_ASYNC,
},


{
.name       = "dump-guest-memory",
.args_type  = "paging:b,protocol:s,begin:i?,end:i?",
.params     = "-p protocol [begin] [length]",
.help       = "dump guest memory to file",
.user_print = monitor_user_noop,
.mhandler.cmd_new = qmp_marshal_input_dump_guest_memory,
},


{
.name       = "netdev_add",
.args_type  = "netdev:O",
.mhandler.cmd_new = qmp_netdev_add,
},


{
.name       = "netdev_del",
.args_type  = "id:s",
.mhandler.cmd_new = qmp_marshal_input_netdev_del,
},


{
.name       = "block_resize",
.args_type  = "device:B,size:o",
.mhandler.cmd_new = qmp_marshal_input_block_resize,
},


{
.name       = "block-stream",
.args_type  = "device:B,base:s?,speed:o?,on-error:s?",
.mhandler.cmd_new = qmp_marshal_input_block_stream,
},

{
.name       = "block-commit",
.args_type  = "device:B,base:s?,top:s,speed:o?",
.mhandler.cmd_new = qmp_marshal_input_block_commit,
},

{
.name       = "block-job-set-speed",
.args_type  = "device:B,speed:o",
.mhandler.cmd_new = qmp_marshal_input_block_job_set_speed,
},

{
.name       = "block-job-cancel",
.args_type  = "device:B,force:b?",
.mhandler.cmd_new = qmp_marshal_input_block_job_cancel,
},
{
.name       = "block-job-pause",
.args_type  = "device:B",
.mhandler.cmd_new = qmp_marshal_input_block_job_pause,
},
{
.name       = "block-job-resume",
.args_type  = "device:B",
.mhandler.cmd_new = qmp_marshal_input_block_job_resume,
},
{
.name       = "block-job-complete",
.args_type  = "device:B",
.mhandler.cmd_new = qmp_marshal_input_block_job_complete,
},
{
.name       = "transaction",
.args_type  = "actions:q",
.mhandler.cmd_new = qmp_marshal_input_transaction,
},


{
.name       = "blockdev-snapshot-sync",
.args_type  = "device:B,snapshot-file:s,format:s?,mode:s?",
.mhandler.cmd_new = qmp_marshal_input_blockdev_snapshot_sync,
},


{
.name       = "drive-mirror",
.args_type  = "sync:s,device:B,target:s,speed:i?,mode:s?,format:s?,"
"on-source-error:s?,on-target-error:s?,"
"granularity:i?,buf-size:i?",
.mhandler.cmd_new = qmp_marshal_input_drive_mirror,
},


{
.name       = "balloon",
.args_type  = "value:M",
.mhandler.cmd_new = qmp_marshal_input_balloon,
},


{
.name       = "set_link",
.args_type  = "name:s,up:b",
.mhandler.cmd_new = qmp_marshal_input_set_link,
},


{
.name       = "getfd",
.args_type  = "fdname:s",
.params     = "getfd name",
.help       = "receive a file descriptor via SCM rights and assign it a name",
.mhandler.cmd_new = qmp_marshal_input_getfd,
},


{
.name       = "closefd",
.args_type  = "fdname:s",
.params     = "closefd name",
.help       = "close a file descriptor previously passed via SCM rights",
.mhandler.cmd_new = qmp_marshal_input_closefd,
},


{
.name       = "add-fd",
.args_type  = "fdset-id:i?,opaque:s?",
.params     = "add-fd fdset-id opaque",
.help       = "Add a file descriptor, that was passed via SCM rights, to an fd set",
.mhandler.cmd_new = qmp_marshal_input_add_fd,
},


{
.name       = "remove-fd",
.args_type  = "fdset-id:i,fd:i?",
.params     = "remove-fd fdset-id fd",
.help       = "Remove a file descriptor from an fd set",
.mhandler.cmd_new = qmp_marshal_input_remove_fd,
},


{
.name       = "query-fdsets",
.args_type  = "",
.help       = "Return information describing all fd sets",
.mhandler.cmd_new = qmp_marshal_input_query_fdsets,
},


{
.name       = "block_passwd",
.args_type  = "device:B,password:s",
.mhandler.cmd_new = qmp_marshal_input_block_passwd,
},


{
.name       = "block_set_io_throttle",
.args_type  = "device:B,bps:l,bps_rd:l,bps_wr:l,iops:l,iops_rd:l,iops_wr:l",
.mhandler.cmd_new = qmp_marshal_input_block_set_io_throttle,
},


{
.name       = "set_password",
.args_type  = "protocol:s,password:s,connected:s?",
.mhandler.cmd_new = qmp_marshal_input_set_password,
},


{
.name       = "expire_password",
.args_type  = "protocol:s,time:s",
.mhandler.cmd_new = qmp_marshal_input_expire_password,
},


{
.name       = "add_client",
.args_type  = "protocol:s,fdname:s,skipauth:b?,tls:b?",
.mhandler.cmd_new = qmp_marshal_input_add_client,
},

{
.name       = "qmp_capabilities",
.args_type  = "",
.params     = "",
.help       = "enable QMP capabilities",
.user_print = monitor_user_noop,
.mhandler.cmd_new = do_qmp_capabilities,
},


{
.name       = "human-monitor-command",
.args_type  = "command-line:s,cpu-index:i?",
.mhandler.cmd_new = qmp_marshal_input_human_monitor_command,
},



{
.name       = "query-version",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_version,
},


{
.name       = "query-commands",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_commands,
},


{
.name       = "query-events",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_events,
},


{
.name       = "query-chardev",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_chardev,
},


{
.name       = "query-block",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_block,
},


{
.name       = "query-blockstats",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_blockstats,
},


{
.name       = "query-cpus",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_cpus,
},


{
.name       = "query-pci",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_pci,
},


{
.name       = "query-kvm",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_kvm,
},


{
.name       = "query-status",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_status,
},


{
.name       = "query-mice",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_mice,
},


{
.name       = "query-vnc",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_vnc,
},


#if defined(CONFIG_SPICE)
{
.name       = "query-spice",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_spice,
},
#endif


{
.name       = "query-name",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_name,
},


{
.name       = "query-uuid",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_uuid,
},


{
.name       = "query-migrate",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_migrate,
},


{
.name       = "migrate-set-capabilities",
.args_type  = "capabilities:O",
.params     = "capability:s,state:b",
.mhandler.cmd_new = qmp_marshal_input_migrate_set_capabilities,
},

{
.name       = "query-migrate-capabilities",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_migrate_capabilities,
},


{
.name       = "query-balloon",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_balloon,
},

{
.name       = "query-block-jobs",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_block_jobs,
},

{
.name       = "qom-list",
.args_type  = "path:s",
.mhandler.cmd_new = qmp_marshal_input_qom_list,
},

{
.name       = "qom-set",
.args_type  = "path:s,property:s,value:q",
.mhandler.cmd_new = qmp_qom_set,
},

{
.name       = "qom-get",
.args_type  = "path:s,property:s",
.mhandler.cmd_new = qmp_qom_get,
},

{
.name       = "nbd-server-start",
.args_type  = "addr:q",
.mhandler.cmd_new = qmp_marshal_input_nbd_server_start,
},
{
.name       = "nbd-server-add",
.args_type  = "device:B,writable:b?",
.mhandler.cmd_new = qmp_marshal_input_nbd_server_add,
},
{
.name       = "nbd-server-stop",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_nbd_server_stop,
},

{
.name       = "change-vnc-password",
.args_type  = "password:s",
.mhandler.cmd_new = qmp_marshal_input_change_vnc_password,
},
{
.name       = "qom-list-types",
.args_type  = "implements:s?,abstract:b?",
.mhandler.cmd_new = qmp_marshal_input_qom_list_types,
},

{
.name       = "device-list-properties",
.args_type  = "typename:s",
.mhandler.cmd_new = qmp_marshal_input_device_list_properties,
},

{
.name       = "query-machines",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_machines,
},

{
.name       = "query-cpu-definitions",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_cpu_definitions,
},

{
.name       = "query-target",
.args_type  = "",
.mhandler.cmd_new = qmp_marshal_input_query_target,
},

{
.name       = "chardev-add",
.args_type  = "id:s,backend:q",
.mhandler.cmd_new = qmp_marshal_input_chardev_add,
},


{
.name       = "chardev-remove",
.args_type  = "id:s",
.mhandler.cmd_new = qmp_marshal_input_chardev_remove,
},


