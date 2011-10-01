/*
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *---------------------------------------------------------------------------
 *
 *	exec.h - supplemental program/script execution
 *	----------------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdnd/exec.c,v 1.11 2002/03/26 15:12:59 hm Exp $
 *
 *      last edit-date: [Tue Mar 26 14:35:46 2002]
 *
 *---------------------------------------------------------------------------*/

#include "isdnd.h"

/*---------------------------------------------------------------------------*
 *	SIGCHLD signal handler
 *---------------------------------------------------------------------------*/
void
sigchild_handler(int sig)
{
	int retstat;
	pid_t pid;

	(void)sig;
	
	if((pid = waitpid(-1, &retstat, WNOHANG)) <= 0)
	{
		log(LL_ERR, "ERROR, sigchild_handler, waitpid: %s", strerror(errno));
		error_exit(1, "ERROR, sigchild_handler, waitpid: %s", strerror(errno));
	}
	else
	{
		if(WIFEXITED(retstat))
		{
			DBGL(DL_PROC, (log(LL_DBG, "normal child (pid=%d) termination, exitstat = %d",
				pid, WEXITSTATUS(retstat))));
		}
		else if(WIFSIGNALED(retstat))
		{
			if(WCOREDUMP(retstat))
				log(LL_WRN, "child (pid=%d) termination due to signal %d (coredump)",
					pid, WTERMSIG(retstat));
			else
				log(LL_WRN, "child (pid=%d) termination due to signal %d",
					pid, WTERMSIG(retstat));
		}
	}

	/* check if hangup required */

	if((pid != 0) &&
	   (pid != ((pid_t)-1)))
	{
	  CEP_FOREACH(cep,&cfg_entry_tab[0])
	  {
	    if(cep->answerpid == pid)
	    {
	      DBGL(DL_PROC, (log(LL_DBG, "disconnect of cdid %d, pid %d",
				 cep->cdid, pid)));

	      cep->answerpid = 0;
	      /**/
	      ev_disconnect_from_user(cep);
	      break;
	    }
	  }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	execute prog as a subprocess and pass an argumentlist
 *---------------------------------------------------------------------------*/
pid_t
exec_prog(const char *prog, const char ** arglist)
{
	char tmp[MAXPATHLEN];
	char path[MAXPATHLEN+1];
	pid_t pid;
	int a;

	snprintf(path, sizeof(path), "%s/%s", ETCPATH, prog);

	arglist[0] = path;

	tmp[0] = '\0';

	for(a=1; arglist[a] != NULL; ++a )
	{
		strlcat(tmp, " ", sizeof(tmp));
		strlcat(tmp, arglist[a], sizeof(tmp));
	}

	DBGL(DL_PROC, (log(LL_DBG, "%s, args:%s", path, tmp)));
	
	switch(pid = fork())
	{
		case -1:		/* error */
			log(LL_ERR, "ERROR: %s", strerror(errno));
			error_exit(1, "ERROR, exec_prog/fork: %s", strerror(errno));
		case 0:			/* child */
			break;
		default:		/* parent */
			return(pid);
	}

	/* this is the child now */

	/*
	 * close files used only by isdnd, e.g.
	 * 1. /dev/i4b
	 * 2. /var/log/isdnd.acct (or similar, when used)
	 * 3. /var/log/isdnd.log (or similar, when used)
	 */
	close(isdnfd);

	if(useacctfile && acctfp)
		fclose(acctfp);

	if(uselogfile && logfp)
		fclose(logfp);

	if(execvp(path, (char * const *)(long)arglist) < 0 )
		_exit(127);

	return(-1);
}

/*---------------------------------------------------------------------------*
 *	run interface up/down script
 *---------------------------------------------------------------------------*/
int
exec_connect_prog(cfg_entry_t *cep, const char *prog, int link_down)
{
	const char *argv[32];
	const char **av = argv;
	char devicename[MAXPATHLEN], addr[100];
	const char *device;
	int s;
	struct ifreq ifr;

	/* the obvious things */
	device = driver_devicename(cep->usrdevicename);
	snprintf(devicename, sizeof(devicename), "%s%d", device, cep->usrdeviceunit);
	*av++ = (const char*)prog;
	*av++ = "-d";
	*av++ = devicename;
	*av++ = "-f";
	*av++ = link_down ? "down" : "up";

	/* try to figure AF_INET address of interface */
	addr[0] = '\0';
	memset(&ifr, 0, sizeof ifr);
	ifr.ifr_addr.sa_family = AF_INET;
	strncpy(ifr.ifr_name, devicename, sizeof(ifr.ifr_name));
	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s >= 0) {
		if (ioctl(s, SIOCGIFADDR, (caddr_t)&ifr) >= 0) {
			struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
			strcpy(addr, inet_ntoa(sin->sin_addr));
			*av++ = "-a";
			*av++ = addr;
		}
		close(s);
	}

	/* terminate argv */
	*av++ = NULL;

	return exec_prog(prog, argv);
}

/*---------------------------------------------------------------------------*
 *	run answeringmachine application
 *---------------------------------------------------------------------------*/
int
exec_answer(cfg_entry_t *cep)
{
	const char *argv[32];
	char devicename[MAXPATHLEN];	
	int pid;
	const char *device;

	if ((cep->answerprog == NULL) ||
	    (cep->answerprog[0] == 0)) {

	    DBGL(DL_PROC, (log(LL_DBG, "No valid answerprog (ignored)")));
	    return (GOOD);
	}
	
	device = driver_devicename(cep->usrdevicename);

	snprintf(devicename, sizeof(devicename), "%si4b%s%d", _PATH_DEV, device,
	    cep->usrdeviceunit);

	argv[0] = cep->answerprog;
	argv[1] = "-D";
	argv[2] = devicename;
	argv[3] = "-d";
	argv[4] = "unknown";
	argv[5] = "-s";
	argv[6] = "unknown";
	argv[7] = NULL;

	/* if destination telephone number avail, add it as argument */
	
	if(*cep->local_phone_incoming.number)
		argv[4] = cep->local_phone_incoming.number;

	/* if source telephone number avail, add it as argument */
	
	if(*cep->real_phone_incoming.number)
		argv[6] = cep->real_phone_incoming.number;

	if(*cep->display)
	{
		argv[7] = "-t";
		argv[8] = cep->display;
		argv[9] = NULL;
	}

	/* exec program */
	
	DBGL(DL_PROC, (log(LL_DBG, "prog=[%s]", cep->answerprog)));
	
	pid = exec_prog(cep->answerprog, argv);
		
	/* store pid */
	if((pid != 0) &&
	   (pid != ((pid_t)-1)))
	{
		cep->answerpid = pid;
		return(GOOD);
	}
	return(ERROR);
}

/*---------------------------------------------------------------------------*
 *	update budget callout/callback statistics counter file
 *---------------------------------------------------------------------------*/
void
upd_callstat_file(char *filename, int rotateflag)
{
	FILE *fp;
	time_t s, l, now;
	int n;
	int ret;

	now = time(NULL);
	
	fp = fopen(filename, "r+");

	if(fp == NULL)
	{
		/* file not there, create it and exit */
		
		log(LL_WRN, "creating %s", filename);

		fp = fopen(filename, "w");
		if(fp == NULL)
		{
			log(LL_ERR, "ERROR, cannot create %s, %s", filename, strerror(errno));
			return;
		}

		ret = fprintf(fp, "%ld %ld 1", now, now);
		if(ret <= 0)
			log(LL_ERR, "ERROR, fprintf failed: %s", strerror(errno));
		
		fclose(fp);
		return;
	}

	/* get contents */
	
	ret = fscanf(fp, "%ld %ld %d", &s, &l, &n);

	/* reset fp */
	
	rewind(fp);
		
	if(ret != 3)
	{
		/* file corrupt ? anyway, initialize */
		
		log(LL_WRN, "initializing %s", filename);

		s = l = now;
		n = 0;
	}

	if(rotateflag)
	{
		struct tm *stmp;
		int dom;

		/* get day of month for last timestamp */
		stmp = localtime(&l);
		dom = stmp->tm_mday;	

		/* get day of month for just now */
		stmp = localtime(&now);
		
		if(dom != stmp->tm_mday)
		{
			FILE *nfp;
			char buf[MAXPATHLEN];

			/* new day, write last days stats */

			sprintf(buf, "%s-%02d", filename, stmp->tm_mday);

			nfp = fopen(buf, "w");
			if(nfp == NULL)
			{
				log(LL_ERR, "ERROR, cannot open for write %s, %s", buf, strerror(errno));
				return;
			}

			ret = fprintf(nfp, "%ld %ld %d", s, l, n);
			if(ret <= 0)
				log(LL_ERR, "ERROR, fprintf failed: %s", strerror(errno));
			
			fclose(nfp);

			/* init new days stats */
			n = 0;
			s = now;

			log(LL_WRN, "rotate %s, new s=%ld l=%ld n=%d", filename, s, l, n);
		}				
	}

	n++;	/* increment call count */

	/*
	 * the "%-3d" is necessary to overwrite any
	 * leftovers from previous contents!
	 */

	ret = fprintf(fp, "%ld %ld %-3d", s, now, n);	

	if(ret <= 0)
		log(LL_ERR, "ERROR, fprintf failed: %s", strerror(errno));
	
	fclose(fp);
}
	
/* EOF */
