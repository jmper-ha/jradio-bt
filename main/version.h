#pragma once

/* The firmware version PONG reports, and what the host shows on its About
 * page. Bumped by hand with a release, together with the git tag of the
 * same number; the build number is for a hotfix on the same minor. The
 * exact commit is in the app descriptor (git describe), printed at boot. */
#define JBT_FW_MAJOR 1U
#define JBT_FW_MINOR 0U
#define JBT_FW_BUILD 1U
