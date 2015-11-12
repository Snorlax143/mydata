#include<stdio.h>

int main()
{
	int node;
	int value;
	scanf("%d", &node);

	for(int i = 0;i < node;i++)
	{
		scanf("%d", &value);
		printf("Nodes[%d].ID = %d\n",i,value);
		scanf("%d", &value);
		printf("Nodes[%d].ci = %d\n",i,value);
		scanf("%d", &value);
		printf("Nodes[%d].si = %d\n",i,value);
		scanf("%d", &value);
		printf("Nodes[%d].Wi = %d\n",i,value);

		for(int j = 0;j < node; j++)
		{
			scanf("%d", &value);
			printf("Nodes[%d].link[%d] = %d\t",i,j,value);
			if(((j+1) % 3 == 0) || (j+1) == node)
				printf("\n");
		}
		for(int j = 0;j < node; j++)
		{
			scanf("%d", &value);
			printf("Nodes[%d].uij[%d] = %d\t",i,j,value);
			if(((j+1) % 3 == 0) || (j+1) == node)
				printf("\n");

		}
		for(int j = 0;j < node; j++)
		{
			scanf("%d", &value);
			printf("Nodes[%d].lij[%d] = %d\t",i,j,value);
			if(((j+1) % 3 == 0) || (j+1) == node)
				printf("\n");
		}
		for(int j = 0;j < node; j++)
		{
			scanf("%d", &value);
			printf("Nodes[%d].pij[%d] = %d\t",i,j,value);
			if(((j+1) % 3 == 0) || (j+1) == node)
				printf("\n");
		}

		for(int j = 0;j < node; j++)
		{
			scanf("%d", &value);
			printf("Nodes[%d].aij[%d] = %d\t",i,j,value);
			if(((j+1) % 3 == 0) || (j+1) == node)
				printf("\n");

		}
		printf("\n");
		
	}
	
}
