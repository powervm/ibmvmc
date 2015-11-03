/* Copyright 2015, IBM
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
class HvHmcLessCmd;
/* standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <endian.h>

/* Typedefs */
#ifndef _HMCLESSTYPES_H
#include "HmcLessTypes.H"
#endif

#ifndef _HVHMCLESSCMD_H
#define _HVHMCLESSCMD_H

const uint16	HvHmcCmdMaxSize		= 4096;
const uint8     HvHmcCmdDesc[3]	        = {0xDE, 0xCA, 0xFF};//DECAFF
const uint8     HvHmcCmdHdrLen       = 32;
class HvHmcLessCmd
{
    public:

    enum Target
    {
	CatHmcRtr               =  0x8000,	// Commands targeted directly to the router
	CatLparConfig           =  0x8001,	// LPAR config commands
	CatLparDlpar            =  0x8002,   // LPAR Dynamic lpar commands
	CatVsp                  =  0x8003,	// VSP commands
	CatCuodAp               =  0x8004,	// CUoD and AP commands
	CatConcMaint            =  0x8005,	// Concurrent maintenance commands
	CatType6                =  0x8006,
	CatType7                =  0x8007,
	CatMax                  =  0x8008
    };

    enum  TypeInd
    {
	Request                 =  0x00,
	Response                =  0x40
    };

    enum  RspInd
    {
	RspNotExpected          =  0x00,
	RspExpected             =  0x80
    };

    enum  Rc
    {
	// Return codes compatable with previous HMC versions
	Good                 = 0x0000, // Successful  (NO_ERROR-x00)
	InvalidOpcode        = 0x8001, // Invalid Opcode (INVALID_COMMAND-x01)
	RsvdHmcNoSuchObject  = 0x8002, // Reserved for v2 NO_SUCH_OBJECT - deprecated
	InvalidData          = 0x8003, // Invalid data (INVALID_DATA-x03)
	RsvdHmcInvalidCmdSeq = 0x8004, // Command not valid due to PHYP Object State - deprecated
	RsvdHmc05            = 0x8005, // suspicious hole in v2 interface - deprecated
	GenericError         = 0x8006, // Use this one when all else fails (COMMAND_FAILED-x06)
	Cancelled            = 0x8007, // Command cancelled due to timeout (COMMAND_TIMEOUT-x07)
	RsvdHmcAsyncStatus   = 0x8008, // Reserved for v2 ASYNC_STATUS - deprecated
	RsvdHmcVttyUpFull    = 0x8009, // Reserved for v2 TERMINAL_BUFFER_FULL - deprecated
	RsvdHmcVttyUpStatus  = 0x8010, // Reserved for v2 TERMINAL_UPSTREAM_STATUS - deprecated

	// Return codes which are new in v3 and not defined in v2 HMC spec
	BufferNotAvailable   = 0x8020, // Buffer not available to queue command to target HMC
	InvalidHmcIndex      = 0x8021, // Invalid HMC index
	InvalidSize          = 0x8022, // Size specified is out of bounds
	InvalidTarget        = 0x8023, // Target specified is not valid   (*NO_SUCH_OBJECT-x02)
	InvalidCategory      = 0x8024, // Category specified is not
        HmcDead              = 0x8025, // Command rejected due to terminating HMC
	PathClosed           = 0x8026, // HMC explicitly closed the command path
	BindEventFailed      = 0x8027, // Binding of event message failed
	InvalidState         = 0x8028, // Invalid state of the session
	SessionMismatch      = 0x8029, // Session mismatch
	InvalidHmcInstance   = 0x802A, // Invalid HMC instance id
	HmcAlreadyOpen       = 0x802B, // This HMC index is already open
	InvalidBufferId      = 0x802C, // Invalid Buffer Id (index)
	InvalidResponse      = 0x802D, // Invalid response
	InvalidSyncRequest   = 0x802E, // Invalid Sync request
    };

// Router Op codes
    enum opcodes
    {
	/* GET / SET COMMANDS */
	ExchangeOps           =  0x0001,  // Open session
	Close                 =  0x0002,  // close session
	cmdRtrEcho            =  0x0103   // Echo test
    };

// Public member function
//Constructor
    HvHmcLessCmd(Target);
// get set functions

    inline uint16 getTarget() const;
    inline void * getDataPtr() const;
    inline uint32 getLength() const;
    inline uint32 getDataLen() const;
    inline uint16 getOpcode() const;
    inline uint8  getParmsByte(uint16 offset) const;

    inline bool   isRequest() const;
    inline bool   isResponseRequested() const;

    void	  setDataLen(uint32 len) ;
    inline void   setOpcode(uint16 val);
    inline void   setType(HvHmcLessCmd::TypeInd ) ;
    inline void   setResp(HvHmcLessCmd::RspInd ) ;

    inline HvHmcLessCmd::Rc 	getRc() const;
    inline void                 setRc(HvHmcLessCmd::Rc);
    inline void                 setParmsByte(uint16 offset , uint8 val);
    inline uint32		getCorrelationToken() const;
    inline void   		setCorrelationToken(uint32 token);
    void dumpHex();

private:

    enum
    {
	RspIndMask = 0x80,
	TypeIndMask = 0x40,
    };

    uint8     xDesc[3];              // Descriptor                        000-002
    uint8      xHeaderLength;          // Length of this command Header     003-003
    uint32     xMessageLength;         // Length of this command Message    004-007
    uint32     xCorrelationToken;      // Correlation token                 008-00B
    uint16     xTarget;                 // Target of the command             00C-00D
    uint8      xRsvd2;		     // Reserved for more flags           00F-00F
    uint8      xFlags;
//    uint8      xRspInd:1;              // a 1 indicates if Response needed  00E-00E
//    uint8      xTypeInd:1;             // a 1 indicates rsp type of command ...
//    uint8      xRsvd1:6;               // Reserved for general flags        ...
    uint32     xDataLength;	     // Length of this command's Data	  010-013
    uint16     xOpcode;                // Target's OpCode                   014-015
    uint16     xRc;                    // Return code                       016-017
    uint64     xRsvd3;		     // Reserved for internal use	  018-01F
    char xPayload[HvHmcCmdMaxSize-HvHmcCmdHdrLen];   // Message
};

//=====================================================================================
// Inline Member Function Definition(s).
//=====================================================================================
uint16 HvHmcLessCmd::getTarget() const
{
    return be16toh((uint16)xTarget);
}

//=====================================================================================
uint32 HvHmcLessCmd::getLength() const
{
    return be32toh(xMessageLength);
}

//=====================================================================================
uint32 HvHmcLessCmd::getDataLen() const
{
    return be32toh(xDataLength);
}

//=====================================================================================
void *  HvHmcLessCmd::getDataPtr() const
{
    return (void*)(((uint8*)this) + xHeaderLength);
}

//=====================================================================================
uint16 HvHmcLessCmd::getOpcode() const
{
    return be16toh(xOpcode);
}

//=====================================================================================
HvHmcLessCmd::Rc HvHmcLessCmd::getRc() const
{
    return (Rc)be16toh((uint16)xRc);
}

//=====================================================================================
bool HvHmcLessCmd::isRequest() const
{
    return (xFlags & TypeIndMask) == Request;
}

//=====================================================================================
bool HvHmcLessCmd::isResponseRequested() const
{
    return (xFlags & RspIndMask) == RspExpected;
}

//=====================================================================================
void  HvHmcLessCmd::setOpcode(uint16 cmd)
{
    xOpcode = htobe16(cmd);
}

//=====================================================================================
void HvHmcLessCmd::setRc(HvHmcLessCmd::Rc rc)
{
    xRc = htobe16((uint16)rc);
}

//=====================================================================================
void HvHmcLessCmd::setType(HvHmcLessCmd::TypeInd x)
{
    xFlags = (xFlags & ~TypeIndMask) | (x & TypeIndMask);
}

//=====================================================================================
void HvHmcLessCmd::setResp(HvHmcLessCmd::RspInd x)
{
    xFlags = (xFlags & ~RspIndMask) | (x & RspIndMask);
}

//=====================================================================================
uint32	HvHmcLessCmd::getCorrelationToken() const
{
    return be32toh(xCorrelationToken);
}

//=====================================================================================
void		HvHmcLessCmd::setCorrelationToken(uint32 token)
{
    xCorrelationToken = htobe32(token);
}

//=====================================================================================
void       HvHmcLessCmd::setParmsByte(uint16 offset , uint8 val)
{
     xPayload[offset] = val;
}

//=====================================================================================
uint8   HvHmcLessCmd::getParmsByte(uint16 offset) const
{
   return xPayload[offset];
}

#endif
