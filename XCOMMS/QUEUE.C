#include <stddef.h>
#include <stdlib.h>
#include "general.h"
#include "unos.h"

class queueclass
{
	protected: struct LISTNODE
						 { LISTNODE huge *next;
							 unsigned char data;
						 };
						 LISTNODE huge *head;
						 LISTNODE huge *tail;
						 int MAXQUEUE;
						 int NUMINQUEUE;
	public:    queueclass(int entries=64);
						 int enqueue(unsigned char dataentry);
						 unsigned char dequeue(void);
						 int isempty(void);
						 int isnotempty(void);
						 int isfull(void);
						 int queuesize(void);
};

queueclass::queueclass(int entries)
{ MAXQUEUE=entries;
	NUMINQUEUE=0;
	head=NULL;
	tail=NULL;
}

int queueclass::enqueue(unsigned char dataentry)
{ LISTNODE *newnode;

	if (NUMINQUEUE==MAXQUEUE) return 1; // if queue full dont put anymore in
	newnode= (LISTNODE *) umalloc(sizeof(LISTNODE));
	if (newnode==NULL) return 2; // out of dynamic memory

	// create node for new entry
	newnode->data= dataentry;
	newnode->next=NULL;

	// insert new node at rear of queue
	if (tail==NULL)
	{ head=newnode; }
	else
	{ tail->next=newnode; }
	tail=newnode;

	NUMINQUEUE++;
	return 0;
}

unsigned char queueclass::dequeue(void)
{ LISTNODE huge *temp;
	unsigned char dataentry;

	if (head==NULL) return NULL; // if queue is empty

	temp=head;
	dataentry=head->data;

	head=head->next;
	if (head==NULL) tail=NULL;

	ufree((char huge *)temp);
	NUMINQUEUE--;
	return dataentry;
}

int queueclass::isempty(void)
{ return (head==NULL);
}

int queueclass::isnotempty(void)
{ return (head!=NULL);
}

int queueclass::isfull(void)
{ return (NUMINQUEUE==MAXQUEUE);
}

int queueclass::queuesize(void)
{ return (NUMINQUEUE);
}
