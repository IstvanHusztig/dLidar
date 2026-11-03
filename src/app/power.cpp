#include "app/power.h"

#include <sys/wait.h>
#include <unistd.h>

bool PowerOff()
{
	pid_t pid = fork();
	if (pid < 0)
		return false;
	if (pid == 0)
	{
		execlp("systemctl", "systemctl", "poweroff", (char *)nullptr);
		_exit(127);
	}
	int status = 0;
	while (waitpid(pid, &status, 0) < 0)
	{
	}
	return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}
