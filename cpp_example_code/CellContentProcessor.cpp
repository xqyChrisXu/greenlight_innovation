#include "CellContentProcessor.h"
#include "CellState.h"
#include "TimeHelper.h"
#include "string.h"
#include "UCL.h"
#include "JPatch.h"
#include "crc.h"
#include "memory_map.h"

CellContentProcessor::CellContentProcessor(SerialMessageSender & externalMessageSender, 
	ZoneManager & zoneManager, CellCommands & cellCommands
	, SystemService & systemService)
	: externalMessageSender(externalMessageSender)
	, zoneManager(zoneManager)
	, cellCommands(cellCommands)
	, systemService(systemService)
{
}

bool CellContentProcessor::CanProcess(uint16_t cmd)
{
	return cmd == CMD_CONTENT_RESPONSE_1 || cmd == CMD_CONTENT_FAIL_1 || CMD_CONTENT_SET;
}
	
bool CellContentProcessor::ProcessMessageLocal(uint16_t cmd, uint8_t* pdata, uint16_t len, uint16_t sequence_in)
{
	//return true;	//block updating for testing
	
	auto & cellState = CellState::GetInstance();
	auto & flashController = FlashController::GetInstance();
	auto clearZone = [this]() {zoneManager.SetZones(NULL, 0); };
	
	if (cmd == CMD_CONTENT_SET)
	{
		uint8_t * pos = pdata;
  
		uint8_t  type;  
		uint32_t final_size;   
		uint32_t dest_hash;

		memcpy((uint8_t*)&type, pos, sizeof(uint8_t));
		pos += sizeof(uint8_t);
  
		memcpy((uint8_t*)&final_size, pos, sizeof(uint32_t));  
		pos += sizeof(uint32_t);

		memcpy((uint8_t*)&dest_hash, pos, sizeof(uint32_t));
		pos += sizeof(uint32_t);
  
		cellCommands.ClearModemSendRetry(true);

		if (cellCommands.SendAck(sequence_in) == 0)
		{   
			if (cellState.GetSettings().suppress_firmware_update == 1 && type == 2)		//ToDo::changed suppress_fimware_update to 0 to test, needs to change back to "1" after finish testing
			{
				return false;
			}

			auto updateData = cellState.GetUpdateData();
			
			if (updateData.state == UPDATE_CONT)
			{
				if (type == updateData.type
				    && final_size == updateData.final_size
				    && dest_hash == updateData.dest_hash)
				{					
					cellCommands.SendContentRequest(clearZone);
					return false;
				}
      
				// If the new stuff is a different type than what is currently downloading
				// then don't get it.  The content catalog will trigger the update again.
				if(type != updateData.type)
				{
					return false;
				}
			}

			cellState.SetUpdateData(updateData);
			updateData = cellState.ClearContentUpdate();						

			updateData.type = type;
			updateData.final_size = final_size;
			updateData.dest_hash = dest_hash;

			externalMessageSender.DiagPrintf(DIAG_COMM,
				2,
				"Content Set Received - CT: %d CSZ: %d CRC: %4.4X [%d]",
				updateData.type,
				updateData.final_size,
				updateData.dest_hash,
				updateData.dest_hash);
        
			//TAG_TO_SERVER_CMD* pcmd = (TAG_TO_SERVER_CMD*)cmd_ack_buf;

			uint8_t* addr = 0;
			int len = 0;
    
			if (updateData.type == 1)
			{
				addr = (uint8_t *)ADDR_FLASH_CONTENT_AREA;
				len = zoneManager.GetZoneBlobSize();
			}
			else if (updateData.type == 2)
			{
				JUMP_BLOCK_DATA* jbapp = (JUMP_BLOCK_DATA*)ADDR_FLASH_JB_ACTIVE_FW_AREA;
				if (jbapp->data_status == 0x33333333)
				{
					addr = (uint8_t *)jbapp->app_address;
					len = jbapp->app_size;
				}
				else
				{
					addr = (uint8_t *)ADDR_FLASH_APP_AREA;
					len = 0;
				}      
			}

			if (len > 0)
			{				
				auto crc = CalcBlockFlipCRCInv((uint32_t*)addr, len);					
				updateData.origin_hash = crc;
			}
    
			cellState.SetUpdateData(updateData);
			
			if (updateData.origin_hash == updateData.dest_hash)
			{
				externalMessageSender.DiagPrintf(DIAG_COMM,
					2,
					"Content up to date - CT: %d CSZ: %d CRC: %4.4X [%d]",
					updateData.type,
					updateData.final_size,
					updateData.dest_hash,
					updateData.dest_hash);

				// Already have the content, send catalog.
				cellCommands.SendContentCatalog();				
			}
			else
			{
				// ack sent, proceed with content requests         				
				cellCommands.SendContentRequest(clearZone);
			}
		}
	}	
	else if (cmd ==  CMD_CONTENT_RESPONSE_1)
	{		
		uint8_t * pos = pdata;
		uint32_t content_id;
		uint32_t content_offset;
		uint32_t compressed_length = 0;
		uint8_t flags = 0;
  
		auto updateData = cellState.GetUpdateData();
		
		updateData.last_response_time = TimeHelper::GetCurrentTime();
		updateData.request_tries = 0;

		int round_trip_time = (updateData.last_response_time - updateData.last_request_time) / 1000;

		// answer received, clear resend variables
		cellCommands.ClearModemSendRetry(true);

		// get the parameters from array
		memcpy((uint8_t*)&content_id, pos, sizeof(uint32_t));
		pos += sizeof(uint32_t);
  
		memcpy((uint8_t*)&content_offset, pos, sizeof(uint32_t));
		pos += sizeof(uint32_t);

		if (content_offset == 0)
		{    
			memcpy((uint8_t*)&updateData.tx_size, pos, sizeof(uint32_t));
			pos += sizeof(uint32_t);

			memcpy((uint8_t*)&flags, pos, sizeof(uint8_t));
			pos += sizeof(uint8_t);

			if (flags & 0x01)
			{
				updateData.compressed = true;
      
				memcpy((uint8_t*)&updateData.uncompressed_size, pos, sizeof(uint32_t));
				pos += sizeof(uint32_t);
			}

			if (flags & 0x02)
			{
				updateData.jpatched = true;
			}
    
			const uint32_t scratch_end_addr = ADDR_FLASH_DATA_AREA + SIZE_FLASH_DATA_AREA;  
  
			// Memory map for jpatch and/or uncompress 
			// |scratch begin..............................scratch end|
			// 1: jpatched
			// |original data    <-                      jpatched data|
			// 2: compressed
			// |original data    <-                    compressed data|
			// 3: compressed and jpatched
			// |compressed&jpatched data    ->           jpatched data|
			// |original data    <-                      jpatched data|

			// By default data at the beginning of the scratch area
			updateData.dest_start_addr = ADDR_FLASH_DATA_AREA;

			if (updateData.jpatched && updateData.compressed)
			{
				// Uncompress at the end downwards
				updateData.dest_uncompressed_addr = scratch_end_addr;
				updateData.dest_uncompressed_alignment = 1;
			}
			else if (updateData.jpatched || updateData.compressed)
			{
				// Put the data at the end of the scratch area.
				updateData.dest_start_addr = (scratch_end_addr - updateData.tx_size) & ~0x7FF;
				updateData.dest_uncompressed_addr = ADDR_FLASH_DATA_AREA;
				updateData.dest_uncompressed_alignment = 0;
			}
		}

		int data_length = len - (pos - pdata);

		// check all necessary conditions: update state, content ID
		if((updateData.state != UPDATE_CONT) || (content_id != updateData.dest_hash))
		{
			// drop the process of the content download.
			cellState.SetUpdateData(updateData);
			cellState.CleanUpdateFct();       
			return false;
		}

		// if offset is not OK just exit without canceling the update process
		if(content_offset != updateData.dest_offset)
		{
			externalMessageSender.DiagPrintfOta(true, DIAG_COMM, 2, "invcontofs");
			cellState.SetUpdateData(updateData);
			return false;
		}

		// program the content in flash
		if(flashController.MemcpyFlash((void *)(updateData.dest_start_addr + updateData.dest_offset), pos, data_length) == NULL)
		{
			cellState.SetUpdateData(updateData);
			// drop the process of the content download
			cellState.CleanUpdateFct();          // clean after update cmd
			externalMessageSender.DiagPrintf(DIAG_COMM, 1, "Flash error");
			return false;
		}

		externalMessageSender.DiagPrintf(DIAG_COMM,
			2,
			"Received Content Block - CID: %4.4X [%d] COFFS: %d (%d BYTES) - %d%%, RTT: %ds", 
			updateData.dest_hash,
			updateData.dest_hash,
			updateData.dest_offset,
			(int)(data_length),
			((updateData.dest_offset + data_length) * 100 / updateData.tx_size),
			round_trip_time);

		updateData.dest_offset += data_length;
    
		if (updateData.dest_offset == updateData.tx_size)
		{
			// content downloaded
			updateData.state = UPDATE_END;
    
			int src_length = updateData.tx_size;

			// Check to see if it needs to be decompressed.
			if(updateData.compressed == true)
			{
				uint32_t size_result;
				uint32_t addr_result;
				if (UCL::Decompression(updateData.dest_start_addr,
					updateData.dest_uncompressed_addr,		//0x8045EF4
					updateData.dest_uncompressed_alignment,
					&size_result,
					&addr_result))
				{
					externalMessageSender.DiagPrintf(DIAG_COMM,
						2,
						"Content decompression failed - CID: %4.4X [%d]", 
						updateData.dest_hash,
						updateData.dest_hash);
					cellState.SetUpdateData(updateData);
					cellState.CleanUpdateFct();          // clean after update cmd
					updateData = cellState.GetUpdateData();
		
				}
				
				// newly decompressed data here
				updateData.dest_start_addr = addr_result;
				src_length = updateData.uncompressed_size;

				externalMessageSender.DiagPrintf(DIAG_COMM,
					2,
					"Decompressed content - CID: %4.4X [%d], SZ: %d, ADDR: %8.8X", 
					updateData.dest_hash,
					updateData.dest_hash,
					size_result,
					addr_result);
			}

			// Check to see if it needs to be jpatched.
			if(updateData.jpatched == true)
			{
				uint32_t addr_original_area = ADDR_FLASH_DATA_AREA; 
				uint32_t size_original_area = SIZE_FLASH_DATA_AREA;
				uint32_t size_patched_result = 0;

				if (updateData.type == 1)
				{
					addr_original_area = ADDR_FLASH_CONTENT_AREA;
					size_original_area = SIZE_FLASH_ZONE_AREA;
				}
				else if (updateData.type == 2)
				{
					addr_original_area = ADDR_FLASH_APP_AREA; 
					size_original_area = SIZE_FLASH_APP_AREA;
				}

				if (JPatch::Patch(addr_original_area, size_original_area, updateData.dest_start_addr, src_length, ADDR_FLASH_DATA_AREA, &size_patched_result))
				{
					externalMessageSender.DiagPrintf(DIAG_COMM, 2, "Content patching failed - CID: [%d]", updateData.dest_hash);
					cellState.SetUpdateData(updateData);
					cellState.CleanUpdateFct();          // clean after update cmd
					return false;
				}

				externalMessageSender.DiagPrintf(DIAG_COMM,
					2,
					"Patched content - CID: [%d], SZ: %d", 
					updateData.dest_hash,
					size_patched_result);

				// The patched data should be at the beginning of the scratch area.
				updateData.dest_start_addr = ADDR_FLASH_DATA_AREA;
			}
    
			uint32_t crc = CalcBlockFlipCRCInv((uint32_t*)ADDR_FLASH_DATA_AREA, updateData.final_size);
						
			if (crc != updateData.dest_hash)
			{
				externalMessageSender.DiagPrintf(DIAG_COMM,
					2,
					"Content CRC check failed - CID: %4.4X [%d] CRC: %4.4X [%d]", 
					updateData.dest_hash, 
					updateData.dest_hash, 
					crc,  
					crc);

				cellState.SetUpdateData(updateData);
				cellState.CleanUpdateFct();           // clean after update cmd
				return false;
			}

			externalMessageSender.DiagPrintf(DIAG_COMM,
				2,
				"Content Update Completed - CID: %4.4X [%d] CRC: %4.4X [%d] DLSZ: %d FSZ: %d, TT: %ds", 
				updateData.dest_hash,
				updateData.dest_hash, 
				crc,
				crc, 
				updateData.dest_offset, 
				updateData.final_size,
				(updateData.last_response_time - updateData.start_time) / 1000);

			updateData.origin_hash = updateData.dest_hash;
     
			if (updateData.type == 1)
			{
				zoneManager.SetZones((uint8_t *)ADDR_FLASH_DATA_AREA, updateData.final_size);
			}
			else if (updateData.type == 2)
			{ 
				JUMP_BLOCK_DATA jbnew = { 0 };
				jbnew.data_status = 0x33333333;
				jbnew.app_address = updateData.dest_start_addr;
				jbnew.app_size = updateData.final_size;
				jbnew.firmware_crc = updateData.dest_hash;
      				
				jbnew.jump_block_data_crc = CalcBlockCRC((uint32_t*)&jbnew, sizeof(JUMP_BLOCK_DATA) / sizeof(uint32_t) - 1);

				flashController.MemcpyFlash((void *)ADDR_FLASH_JB_NEW_FW_AREA, &jbnew, sizeof(JUMP_BLOCK_DATA));

				externalMessageSender.DiagPrintf(DIAG_COMM, 0, "Firmware downloaded, restarting... CID: [%d]", updateData.dest_hash);
				
				osDelay(5000);
				
				systemService.RebootSystem(true, REBOOT_FIRMWARE_UPDATE);
				
				return false;
			}
			
			cellState.SetUpdateData(updateData);
			cellCommands.SendContentCatalog();
		}
		else
		{
			cellState.SetUpdateData(updateData);														
			cellCommands.SendContentRequest(clearZone);
		}	
	}
	else if (cmd == CMD_CONTENT_FAIL_1)
	{
		uint32_t content_id;

		//Todo:: not sure why we need it here
		// points to the tx buffer  
		//TAG_TO_SERVER_CMD* pcmd = (TAG_TO_SERVER_CMD*)cmd_ack_buf;

		// answer received, clear resend variables
		cellCommands.ClearModemSendRetry(true);
  
		// get the parameters from array
		memcpy((uint8_t*)&content_id, pdata, sizeof(content_id));

		cellState.CleanUpdateFct();         // clean after update cmd
		
		externalMessageSender.DiagPrintf(DIAG_COMM, 1, "Content Download Failed - CID [%d]", cellState.GetUpdateData().dest_hash);

		cellState.ClearContentUpdate();
		
	}
	
	return false;
}

CellContentProcessor::~CellContentProcessor()
{
}
