class queueclass
{
	protected: struct LISTNODE
						 { LISTNODE *next;
							 unsigned char data;
						 };
						 LISTNODE *head;
						 LISTNODE *tail;
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

/* extern queueclass::queueclass(int entries=64);
extern int queueclass::enqueue(void *dataentry);
extern void *queueclass::dequeue(void);
extern int queueclass::isempty(void);
extern int queueclass::isnotempty(void);
extern int queueclass::isfull(void);
extern int queueclass::queuesize(void);
*/