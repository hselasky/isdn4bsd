/* $FreeBSD$ */
/*-
 * Copyright (c) 1999, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_ing.c - isdn4bsd B-channel to netgraph driver
 *	-------------------------------------------------
 *
 *	last edit-date: [Sat Mar  9 14:09:53 2002]
 *
 *---------------------------------------------------------------------------*/

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#else
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/malloc.h>
#include <net/if.h>
#endif

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

#define I4BINGMAXQLEN	50		/* max queue length */

/* initialized by L4 */

struct ing_softc {
	int		sc_unit;	/* unit number			*/
	int		sc_state;	/* state of the interface	*/
	call_desc_t	*sc_cdp;	/* ptr to call descriptor	*/

	fifo_translator_t *sc_fifo_translator; /* fifo translator       */
	
#if I4B_ACCOUNTING
	struct i4b_accounting sc_accounting;
#endif	
	struct _ifqueue xmitq_hipri;	/* hi-priority transmit queue */
	struct _ifqueue xmitq;		/* transmit queue */
		
	node_p		node;		/* back pointer to node */
	char		nodename[64];	/* store our node name */
	hook_p  	debughook;
	hook_p  	hook;	

	uint32_t	flags;

} ing_softc[NI4BING];

enum ing_states {
	ST_IDLE,			/* initialized, ready, idle	*/
	ST_DIALING,			/* dialling out to remote	*/
	ST_CONNECTED			/* connected to remote		*/
};

/* ========= NETGRAPH ============= */

#define NG_ING_NODE_TYPE	"i4bing"	/* node type name */
#define NGM_ING_COOKIE		947513046	/* node type cookie */

/* Hook names */
#define NG_ING_HOOK_DEBUG	"debug"
#define NG_ING_HOOK_RAW		"rawdata"

/* Netgraph commands understood by this node type */
enum {
	NGM_ING_SET_FLAG = 1,
	NGM_ING_GET_STATUS,
};

/* This structure is returned by the NGM_ING_GET_STATUS command */
struct ngingstat {
	u_int   packets_in;	/* packets in from downstream */
	u_int   packets_out;	/* packets out towards downstream */
};

/*
 * This is used to define the 'parse type' for a struct ngingstat, which
 * is bascially a description of how to convert a binary struct ngingstat
 * to an ASCII string and back.  See ng_parse.h for more info.
 *
 * This needs to be kept in sync with the above structure definition
 */
#define NG_ING_STATS_TYPE_INFO	{				\
	  { "packets_in",	&ng_parse_int32_type	},	\
	  { "packets_out",	&ng_parse_int32_type	},	\
	  { NULL, NULL },				       	\
}

/*
 * This section contains the netgraph method declarations for the
 * sample node. These methods define the netgraph 'type'.
 */

static ng_constructor_t	ng_ing_constructor;
static ng_rcvmsg_t	ng_ing_rcvmsg;
static ng_shutdown_t	ng_ing_shutdown;
static ng_newhook_t	ng_ing_newhook;
static ng_connect_t	ng_ing_connect;
static ng_rcvdata_t	ng_ing_rcvdata;
static ng_disconnect_t	ng_ing_disconnect;

/* Parse type for struct ngingstat */
static const struct
	ng_parse_struct_field ng_ing_stat_type_fields[] =
	NG_ING_STATS_TYPE_INFO;

static const struct ng_parse_type ng_ing_stat_type = {
	&ng_parse_struct_type,
	&ng_ing_stat_type_fields
};

/* List of commands and how to convert arguments to/from ASCII */

static const struct ng_cmdlist ng_ing_cmdlist[] = {
	{
		NGM_ING_COOKIE,
		NGM_ING_GET_STATUS,
		"getstatus",
		NULL,
		&ng_ing_stat_type,
	},
	{
		NGM_ING_COOKIE,
		NGM_ING_SET_FLAG,
		"setflag",
		&ng_parse_int32_type,
		NULL
	},
	{ 0 }
};

/* Netgraph node type descriptor */
static struct ng_type typestruct = {
	.version =	NG_ABI_VERSION,
	.name =		NG_ING_NODE_TYPE,
	.constructor =	ng_ing_constructor,
	.rcvmsg =	ng_ing_rcvmsg,
	.shutdown =	ng_ing_shutdown,
	.newhook =	ng_ing_newhook,
	.connect =	ng_ing_connect,
	.rcvdata =	ng_ing_rcvdata,
	.disconnect =	ng_ing_disconnect,
	.cmdlist =	ng_ing_cmdlist,
};

NETGRAPH_INIT_ORDERED(ing, &typestruct, SI_SUB_DRIVERS, SI_ORDER_ANY);

/*===========================================================================*
 *			DEVICE DRIVER ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	interface attach routine at kernel boot time
 *---------------------------------------------------------------------------*/
static void
i4bingattach(void *dummy)
{
	struct ing_softc *sc = ing_softc;

	int i;
	int ret;

	printf("i4bing: %d i4b NetGraph ISDN B-channel device(s) attached\n", NI4BING);
	
	for(i=0; i < NI4BING; sc++, i++)
	{
		sc->sc_unit = i;
		
		NDBGL4(L4_DIALST, "setting dial state to ST_IDLE");

		sc->sc_state = ST_IDLE;
		
#if I4B_ACCOUNTING
		I4B_ACCOUNTING_INIT(&sc->sc_accounting);
#endif

		/* setup a netgraph node */

		if ((ret = ng_make_node_common(&typestruct, &sc->node)))
		{
			printf("ing: ng_make_node_common, ret = %d\n!", ret);
		}

		/* name the netgraph node */

		snprintf(sc->nodename, sizeof(sc->nodename), "%s%d", NG_ING_NODE_TYPE, sc->sc_unit);
		if((ret = ng_name_node(sc->node, sc->nodename)))
		{
			printf("ing: ng_name node, ret = %d\n!", ret);
			NG_NODE_UNREF(sc->node);
			break;
		}

		NG_NODE_SET_PRIVATE(sc->node, sc);

		sc->xmitq.ifq_maxlen = IFQ_MAXLEN;
		sc->xmitq_hipri.ifq_maxlen = IFQ_MAXLEN;
	}
}
SYSINIT(i4bingattach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4bingattach, NULL);

/*===========================================================================*
 *			ISDN INTERFACE ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	this routine is used to give a feedback from userland daemon
 *	in case of dial problems
 *---------------------------------------------------------------------------*/
static void
ing_response_to_user(msg_response_to_user_t *mrtu)
{
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called each time a packet is received or transmitted.
 *	It should be used to implement an activity timeout mechanism.
 *---------------------------------------------------------------------------*/
static void
ing_activity(struct ing_softc *sc)
{
	if (sc->sc_cdp) {
	    sc->sc_cdp->last_active_time = SECOND;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
ing_put_mbuf(struct fifo_translator *__f, struct mbuf *m)
{
	register struct ing_softc *sc = __f->L5_sc;
	int error;

	ing_activity(sc);

#if I4B_ACCOUNTING
	sc->sc_accounting.sc_inb += m->m_pkthdr.len;
	sc->sc_accounting.sc_inpkt++;
#endif

	m->m_pkthdr.rcvif = NULL;

	NG_SEND_DATA_ONLY(error, sc->hook, m);

	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame (mbuf)
 *---------------------------------------------------------------------------*/
static struct mbuf *
ing_get_mbuf(struct fifo_translator *__f)
{
	register struct ing_softc *sc = __f->L5_sc;
	register struct mbuf *m;

	_IF_DEQUEUE(&sc->xmitq_hipri, m);

	if(m == NULL)
	{
		_IF_DEQUEUE(&sc->xmitq, m);
	}

	if(m)
	{
#if I4B_ACCOUNTING
		sc->sc_accounting.sc_outb += m->m_pkthdr.len;
		sc->sc_accounting.sc_outpkt++;
#endif
		ing_activity(sc);
	}

	return m;
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
static fifo_translator_t *
ing_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	     struct i4b_protocol *pp, uint32_t driver_type, 
	     uint32_t driver_unit, call_desc_t *cd)
{
	struct ing_softc *sc = &ing_softc[driver_unit];

	if(!pp)
	{
	  return (driver_unit < NI4BING) ? 
	    sc->sc_fifo_translator : FT_INVALID;
	}

	sc->sc_fifo_translator = f;

#if I4B_ACCOUNTING
	I4B_ACCOUNTING_UPDATE(&sc->sc_accounting,
			      sc->sc_fifo_translator,
			      DRVR_ING,
			      driver_unit);
#endif
	if(pp->protocol_1)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF = ing_put_mbuf;
	  f->L5_GET_MBUF = ing_get_mbuf;

	  sc->sc_cdp = cd;

	  NDBGL4(L4_DIALST, "ing%d: setting dial state to ST_CONNECTED",
		 driver_unit);

	  sc->sc_state = ST_CONNECTED;
	}
	else
	{
	  /* not connected */

	  sc->sc_cdp = NULL;

	  NDBGL4(L4_DIALST, "ing%d: setting dial state to ST_IDLE",
		 driver_unit);

	  sc->sc_state = ST_IDLE;
	}

	return f;
}
I4B_REGISTER(DRVR_ING, ing_setup_ft, ing_response_to_user);

/*===========================================================================*
 *			NETGRAPH INTERFACE ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 * It is not possible or allowable to create a node of this type.
 * If the hardware exists, it will already have created it.
 *---------------------------------------------------------------------------*/
static int
ng_ing_constructor(node_p node)
{
	return(EINVAL);
}

/*---------------------------------------------------------------------------*
 * Give our ok for a hook to be added...
 * Add the hook's private info to the hook structure.
 *---------------------------------------------------------------------------*/
static int
ng_ing_newhook(node_p node, hook_p hook, const char *name)
{
	struct ing_softc *sc = NG_NODE_PRIVATE(node);
	int error = EINVAL;

	SC_LOCK(f,sc->sc_fifo_translator);

	/*
	 * check if it's our friend the debug hook
	 */
	if(strcmp(name, NG_ING_HOOK_DEBUG) == 0)
	{
		NG_HOOK_SET_PRIVATE(hook, NULL); /* paranoid */
		sc->debughook = hook;
		error = 0;
		goto done;
	}
	/*
	 * Check for raw mode hook.
	 */
	if(strcmp(name, NG_ING_HOOK_RAW) == 0)
	{
		NG_HOOK_SET_PRIVATE(hook, sc);
		sc->hook = hook;
		error = 0;
		goto done;
	}
 done:
	SC_UNLOCK(f);

	return error;
}

/*---------------------------------------------------------------------------*
 * Get a netgraph control message.
 * Check it is one we understand. If needed, send a response.
 * We could save the address for an async action later, but don't here.
 * Always free the message.
 * The response should be in a malloc'd region that the caller can 'free'.
 * A response is not required.
 *---------------------------------------------------------------------------*/
static int
ng_ing_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	struct ing_softc *sc = NG_NODE_PRIVATE(node);

	struct ng_mesg *resp = NULL;
	int error = 0;
	struct ng_mesg *msg;

	NGI_GET_MSG(item, msg);

	SC_LOCK(f,sc->sc_fifo_translator);

	if(msg->header.typecookie == NGM_GENERIC_COOKIE)
	{
		switch(msg->header.cmd)
		{
			case NGM_TEXT_STATUS:
			{
				char *arg;
				char *p;
				int pos = 0;

				NG_MKRESPONSE(resp, msg, sizeof(struct ng_mesg) + NG_TEXTRESPONSE, M_NOWAIT);

				if (resp == NULL)
				{
					error = ENOMEM;
					break;
				}
				arg = (char *) resp->data;

				switch(sc->sc_state)
				{
			    		case ST_IDLE:
						p = "idle";
						break;
				    	case ST_DIALING:
						p = "dialing";
						break;
				    	case ST_CONNECTED:
						p = "connected";
						break;
				    	default:
						p = "???";
						break;
			    	}

				pos = sprintf(arg, "state = %s (%d)\n", p, sc->sc_state);
#if I4B_ACCOUNTING
				pos += sprintf(arg + pos, "%d bytes in, %d bytes out\n",
					       sc->sc_accounting.sc_inb,
					       sc->sc_accounting.sc_outb);
				pos += sprintf(arg + pos, "%d pkts in, %d pkts out\n",
					       sc->sc_accounting.sc_inpkt,
					       sc->sc_accounting.sc_outpkt);
#endif
				resp->header.arglen = pos + 1;
				break;
			}

			default:
				error = EINVAL;
				break;
		}
	}
	else if(msg->header.typecookie == NGM_ING_COOKIE)
	{
		switch (msg->header.cmd)
		{
#if I4B_ACCOUNTING
			case NGM_ING_GET_STATUS:
			{
				struct ngingstat *stats;

				NG_MKRESPONSE(resp, msg, sizeof(*stats), M_NOWAIT);

				if (!resp)
				{
					error = ENOMEM;
					break;
				}

				stats = (struct ngingstat *) resp->data;
				stats->packets_in = sc->sc_accounting.sc_inpkt;
				stats->packets_out = sc->sc_accounting.sc_outpkt;
				break;
			}
#endif
			case NGM_ING_SET_FLAG:
				if (msg->header.arglen != sizeof(uint32_t))
				{
					error = EINVAL;
					break;
				}
				sc->flags = *((uint32_t *) msg->data);
				break;

			default:
				error = EINVAL;		/* unknown command */
				break;
		}
	}
	else
	{
		error = EINVAL;			/* unknown cookie type */
	}

	SC_UNLOCK(f);

	/* Take care of synchronous response, if any */
	NG_RESPOND_MSG(error, node, item, resp);
	/* Free the message and return */
	NG_FREE_MSG(msg);
	return(error);
}

/*---------------------------------------------------------------------------*
 * get data from another node and transmit it out on a B-channel
 *---------------------------------------------------------------------------*/
static int
ng_ing_rcvdata(hook_p hook, item_p item)
{
	struct ing_softc *sc = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));
	struct _ifqueue *xmitq_p;
	struct mbuf *m;
	struct ng_tag_prio *ptag;
	
	NGI_GET_M(item, m);
	NG_FREE_ITEM(item);

	if(NG_HOOK_PRIVATE(hook) == NULL)
	{
		NG_FREE_M(m);
		return(ENETDOWN);
	}

	/*
	 * Now queue the data for when it can be sent
	 */
	ptag = (void *)m_tag_locate(m, NGM_GENERIC_COOKIE, NG_TAG_PRIO, NULL);

	if (ptag && (ptag->priority > NG_PRIO_CUTOFF))
	{
		xmitq_p = (&sc->xmitq_hipri);
	}
	else
	{
		xmitq_p = (&sc->xmitq);
	}

	SC_LOCK(f,sc->sc_fifo_translator);

	if(_IF_QFULL(xmitq_p))
	{
#if 0
		_IF_DROP(xmitq_p);
#endif
	}
	else
	{
		_IF_ENQUEUE(xmitq_p, m);
		m = NULL;
	}

	if(f)
	{
		/* connected */
		L1_FIFO_START(sc->sc_fifo_translator);
	}
	else
	{
		/* not connected */
		i4b_l4_dialout(DRVR_ING, sc->sc_unit);
		sc->sc_state = ST_DIALING;
	}

	SC_UNLOCK(f);

	if(m)
	{
		NG_FREE_M(m);
		return(ENOBUFS);
	}
	return (0);
}

/*---------------------------------------------------------------------------*
 * Do local shutdown processing..
 * If we are a persistant device, we might refuse to go away, and
 * we'd only remove our links and reset ourself.
 *---------------------------------------------------------------------------*/
static int
ng_ing_shutdown(node_p node)
{
	struct ing_softc *sc = NG_NODE_PRIVATE(node);
	int	ret;

	NG_NODE_UNREF(node);

	SC_LOCK(f,sc->sc_fifo_translator);

	if((ret = ng_make_node_common(&typestruct, &sc->node)))
	{
		printf("%s: ng_make_node_common, ret = %d\n!", 
		       __FUNCTION__, ret);
	}

	/* name the netgraph node */
	snprintf(sc->nodename, sizeof(sc->nodename), "%s%d", NG_ING_NODE_TYPE, sc->sc_unit);
	if((ret = ng_name_node(sc->node, sc->nodename)))
	{
		printf("%s: ng_name node, ret = %d\n!", 
		       __FUNCTION__, ret);
		NG_NODE_UNREF(sc->node);
	}
	else
	{
		NG_NODE_SET_PRIVATE(sc->node, sc);
	}

	SC_UNLOCK(f);

	return (0);
}

/*---------------------------------------------------------------------------*
 * This is called once we've already connected a new hook to the other node.
 *---------------------------------------------------------------------------*/
static int
ng_ing_connect(hook_p hook)
{
	/* probably not at splnet, force outward queueing */
	NG_HOOK_FORCE_QUEUE(NG_HOOK_PEER(hook));
	return (0);
}

/*
 * Hook disconnection
 *
 * For this type, removal of the last link destroys the node
 */
static int
ng_ing_disconnect(hook_p hook)
{
	struct ing_softc *sc = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));

	SC_LOCK(f,sc->sc_fifo_translator);

	if(NG_HOOK_PRIVATE(hook))
	{
	}
	else
	{
		sc->debughook = NULL;
	}

	SC_UNLOCK(f);

	return (0);
}
