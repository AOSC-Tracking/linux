===========================
Apple File System for Linux
===========================

The Apple File System (APFS) is the copy-on-write filesystem currently used on
all Apple devices. This module provides a degree of experimental support on
Linux.

To help test write support, a set of userland tools is also under development.
The git tree can be retrieved from https://github.com/eafer/apfsprogs.git.

Known limitations
=================

This module is the result of reverse engineering and testing has been limited.
If you make use of the write support, there is a real risk of data corruption.
Please report any issues that you find.

Writes to fusion drives are not currently supported.
Encryption is also not yet implemented even in read-only mode.

Reporting bugs
==============

If you encounter any problem, the first thing you should do is run (as root)::

	dmesg | grep -i apfs

to see all the error messages. If that doesn't help you, please report the
issue via email at lore.kernel.org.

Mount
=====

Like all filesystems, apfs is mounted with::

	mount [-o options] device dir

where ``device`` is the path to your device file or filesystem image, and
``dir`` is the mount point. The following options are accepted:

============   =================================================================
vol=n	       Volume number to mount. The default is volume 0.

snap=label     Volume snapshot to mount (in read-only mode).

tier2=path     Path to the tier 2 device. For fusion drives only.

uid=n, gid=n   Override on-disk inode ownership data with given uid/gid.

cknodes	       Verify the checksum on all metadata nodes. Right now this has a
	       severe performance cost, so it's not recommended.

readwrite      Enable the experimental write support. This may corrupt your
	       container.
============   =================================================================

So for instance, if you want to mount volume number 2, and you want the metadata
to be checked, you should run (as root)::

	mount -o cknodes,vol=2 device dir

To unmount it, run::

	umount dir

Credits
=======

Originally written by Ernesto A. Fernández <ernesto@corellium.com>, with
several contributions from Gabriel Krisman Bertazi <krisman@collabora.com>,
Arnaud Ferraris <arnaud.ferraris@collabora.com> and Stan Skowronek
<skylark@disorder.metrascale.com>. For attribution details see the historical
git tree at https://github.com/eafer/linux-apfs.git and
https://github.com/linux-apfs/linux-apfs-rw.

The module was ported by Ethan Carter Edwards <ethan@ethancedwards.com> to
mainline and submitted upstream in 2025.

Work was first based on reverse engineering done by others [1]_ [2]_, and later
on the (very incomplete) official specification [3]_. Some parts of the code
imitate the ext2 module, and to a lesser degree xfs, udf, gfs2 and hfsplus.

.. [1] Hansen, K.H., Toolan, F., Decoding the APFS file system, Digital
   Investigation (2017), https://dx.doi.org/10.1016/j.diin.2017.07.003
.. [2] https://github.com/sgan81/apfs-fuse
.. [3] https://developer.apple.com/support/apple-file-system/Apple-File-System-Reference.pdf
