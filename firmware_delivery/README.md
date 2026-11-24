# 褰撳墠鍥轰欢浜や粯

## 褰撳墠椋炶鍩虹嚎

| 鐢ㄩ€?| 闀滃儚 | 鐑у綍鍦板潃 |
|---|---|---|
| **姝ｅ父椋炶锛堝敮涓€鐜拌锛?* | [Robin_24G_Car_flight_control_baseline.bin](candidates/Robin_24G_Car_flight_control_baseline.bin) | `0x4000` |
| 鏃?OTA bootloader | [Robin_24G_Bootloader_no_ota.bin](candidates/Robin_24G_Bootloader_no_ota.bin) | `0x0` |
| 閬ユ帶鍣ㄦ仮澶嶉暅鍍?| [Robin_24G_Remote_logkey.bin](candidates/Robin_24G_Remote_logkey.bin) | `0x0` |
| 鍏酱鏍″噯缁存姢闀滃儚 | [269A 缁存姢鍥轰欢](daily/Robin_24G_Car_maint_269a.bin) | `0x4000` |
| 瀹氱偣娓╄ˉ鍦伴潰璇婃柇 | [Robin_24G_Car_ground_baro_fixed_bdt.bin](candidates/Robin_24G_Car_ground_baro_fixed_bdt.bin) | `0x4000` |

褰撳墠椋炶鍩虹嚎锛?
```text
婧愮爜鎻愪氦锛?9215e9
鏋勫缓寮€鍏筹細-DC_FLIGHT_CONTROL_EN=1
姘斿帇锛欳_FLIGHT_BARO_EN=0锛堥粯璁ゅ叧闂級
瀛楄妭鏁帮細24516
SHA256锛欳4DE867D9C8C0B0937382EABF50DAFD307FD43BDA2994A98DE43473FEAAB07F2
鐑у綍鍦板潃锛?x4000
```

璇ラ暅鍍忓凡閫氳繃涓绘満鍥炲綊鍜屾甯搁琛屾瀯寤烘鏌ャ€?*瀹為璁板綍灞炰簬褰掓。鐨?`fb28cc3` 闀滃儚**锛屼笉鏄繖浠芥柊鍩虹嚎銆?
鍘?`fb28cc3` 瀹為闀滃儚褰掓。锛?
```text
archive/Robin_24G_Car_flight_control_fb28cc3.bin
24948 瀛楄妭
B9B98FF5B82105C150BFDD10078B9DD07987EBD3B0F065668E795610A6AF010C
```

鍏朵粬姘斿帇椋炶鍊欓€夊拰閲嶅鐨勫叧闂増姘斿帇瀹炴祴浠跺凡绉诲埌 [褰掓。鐩綍](archive/)锛屼笉鍐嶄綅浜庡綋鍓嶄氦浠樼洰褰曘€?
## 鏍￠獙

鍦ㄤ粨搴撴牴鐩綍杩愯锛?
```powershell
python firmware_delivery\verify_hashes.py
```

瀹屾暣鏂囦欢銆佸湴鍧€鍜屽搱甯岃 [BIN 娓呭崟](INVENTORY.md)銆傚師濮嬪惎鍔ㄣ€侀仴鎺у拰鏈鸿浇鍥轰欢淇濈暀鍦?[firmware_baseline](../firmware_baseline/)銆?
## 璇存槑

- 1 Hz 姘斿帇娴嬮噺鍜屽畾鐐规俯琛ヤ唬鐮佷繚鐣欙紝榛樿涓嶅惎鐢紱
- 姝ｅ父椋炶浠嶄娇鐢?200 Hz 濮挎€佹帶鍒躲€佷繚鎶ら€昏緫銆佹贩鎺у拰鎵嬪姩娌归棬锛?- 鏈疆鏈儳褰曘€佹湭鎿嶄綔瀹炴満銆?