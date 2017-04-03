/* Circular Buffer
 * 
 * PURPOSE: Class implementation of a circular buffer
 * 
 * FUNCTIONS
 * - size(): Get working size of buffer 
 * - pop():  Get first element and remove from buffer
 * - push(): Add element to end of buffer
 * - peek(): Get value of first element
 * - printitems(): Print all items in current buffer
 * 
 * AUTHOR: Justin Lam
 * LAST EDITED: August 24, 2016
 */

#pragma once

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#define BUFFER_SIZE 25

class CircularBuffer
{
private:
	// Global variables for circular queue
	float cbuffer[BUFFER_SIZE];
	unsigned int cQueueStart;
	unsigned int cQueueEnd;
	float fromBuffer;

	// Returns the first working index
	unsigned int getStartInd() {
		return cQueueStart % BUFFER_SIZE;
	}

	// Returns the last working index
	unsigned int getEndInd() {
		return cQueueEnd % BUFFER_SIZE;
	}

public:
	// Constructor
	CircularBuffer() {
		cQueueStart = 0;
		cQueueEnd = 0;
		fromBuffer = 0;
	}

	// Get working size of circular buffer
	int size()
	{
		int n = cQueueEnd - cQueueStart;

		if (n > BUFFER_SIZE)
			n = BUFFER_SIZE;

		return n;
	}

	// Get first element of circular buffer and remove it from queue.
	float pop()
	{
		float dataOut = 0.0;

		if (size() > 0)
		{
			dataOut = cbuffer[getStartInd()];
			cbuffer[getStartInd()] = 0.0;
			cQueueStart++;
		}

		return dataOut;
	}

	// Get first element of circular buffer without removing from queue.
	float peek()
	{
		return cbuffer[getStartInd()];;
	}

	// Add data to end of circular buffer
	void push(float dataIn)
	{
		cbuffer[getEndInd()] = dataIn;
		cQueueEnd++;
	}

	// Prints everything inside buffer
	void printitems()
	{
		int index = getStartInd();

		do {
			std::cout << cbuffer[index % BUFFER_SIZE] << ", ";
			index++;
		} while (index < size());

		std::cout << std::endl;
	}
};

#endif
