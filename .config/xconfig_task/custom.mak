## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28L linker.cmd package/cfg/task_p28L.o28L

linker.cmd: package/cfg/task_p28L.xdl
	$(SED) 's"^\"\(package/cfg/task_p28Lcfg.cmd\)\"$""\"C:/ccsv5/task28_TMS320F28027/.config/xconfig_task/\1\""' package/cfg/task_p28L.xdl > $@
