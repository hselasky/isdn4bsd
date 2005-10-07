/*
 * Copyright (c) 1999 Hellmuth Michaelis. All rights reserved.
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
 *	display.h - isdnphone - some display operations
 *      ===============================================
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 *	init curses fullscreen display
 *---------------------------------------------------------------------------*/
static void
init_mainw(void)
{
	char buffer[512];
	
	initscr();			/* curses init */

	if((COLS < 80) || (LINES < 24))
		exit_fatal("ERROR, minimal screensize must be 80x24, "
			   "is %dx%d, terminating!", COLS, LINES);

	if((main_w = newwin(MW_HEIGHT, MW_WIDTH, MW_ROW, MW_COL)) == NULL)
		exit_fatal("ERROR, curses init main window, terminating!");

	if((dbg_w = newwin(DB_HGT, DB_WID, DB_ROW, DB_COL)) == NULL)
	  exit_fatal("ERROR, curses init debug window, terminating!");

	scrollok(dbg_w, TRUE);

	raw();					/* raw input */
	noecho();				/* do not echo input */

	keypad(stdscr, TRUE);			/* use special keys */
	keypad(main_w, TRUE);			/* use special keys */

	box(main_w, 0, 0);

	sprintf(buffer, "isdnphone %d.%d ", I4B_VERSION, I4B_REL);

	wstandout(main_w);
	mvwaddstr(main_w, 0,  (MW_WIDTH / 2) - (strlen(buffer) / 2), buffer);
	wstandend(main_w);

	mvwaddstr(main_w, 0, MW_WIDTH-8, "(h)elp");

	mvwaddstr(main_w, MW_STATEY, MW_STATEX, "  state: ");
	wmove(main_w, MW_STATEY+1, 1);
	whline(main_w, 0, MW_WIDTH-2);
	
	mvwaddstr(main_w, MW_NUMY, MW_NUMX, " number: ");
	wmove(main_w, MW_NUMY+1, 1);
	whline(main_w, 0, MW_WIDTH-2);
	
	mvwaddstr(main_w, MW_MSGY, MW_MSGX, "message: ");

	curses_ready = 1;

	update_state(EV_UPDATE,
	       NULL);

	return;
}

static const char help[];

/*---------------------------------------------------------------------------*
 *	curses help-window
 *---------------------------------------------------------------------------*/
static void
do_help(void)
{
  if(help_w)
  {
    /* delete the help-window */

    wclear(help_w);
    wrefresh(help_w);
    delwin(help_w);

    /* re-display the original lower window contents */

    touchwin(main_w);
    wrefresh(main_w);

    help_w = 0;
  }
  else
  {
    help_w = newwin(24 /* lines */ , 64 /* columns */,
		    0 /* line  */ , (80-64)/2 /* column */);

    if(help_w)
    {
      mvwaddstr(help_w,0,0,&help[0]);

      wmove(help_w, 2 /*y*/, 0/*x*/);
      whline(help_w, 0, 64);

      wmove(help_w, 0 /*y*/, 5/*x*/);
      wvline(help_w, 0, 24);

      box(help_w, 0, 0);

      wrefresh(help_w);
    }
  }
  return;
}
/* EOF */
