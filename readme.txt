#¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿
¿¿¿¿¿¿¿¿¿¿¿¿
/**
 ***************************************************************************************************
 * ÊµÑé¼ò½é
 * ÊµÑéÃû³Æ : lwIP+FreeRTOS²Ù×÷ÏµÍ³ÒÆÖ² ÊµÑé
 * ÊµÑéÆ½Ì¨ : ÕýµãÔ­×Ó Ì½Ë÷ÕßF407¿ª·¢°å
 * ÊµÑéÄ¿µÄ : Ñ§Ï°lwIP+FreeRTOS²Ù×÷ÏµÍ³ÒÆÖ²

 ***************************************************************************************************
 * Ó²¼þ×ÊÔ´¼°Òý½Å·ÖÅä
 * 1 LEDµÆ
     DS0£¨RED£©     : LED0 - PE0
 * 2 ´®¿Ú1 (PA9/PA10Á¬½ÓÔÚ°åÔØUSB×ª´®¿ÚÐ¾Æ¬CH340ÉÏÃæ)
 * 3 ÕýµãÔ­×Ó 2.8/3.5/4.3/7/10´çTFTLCDÄ£¿é(½öÏÞMCUÆÁ£¬16Î»8080²¢¿ÚÇý¶¯)
 * 4 ETH,YT8521ÍøÂçÐ¾Æ¬
        ETH_MDIO -------------------------> PA2
        ETH_MDC --------------------------> PC1
        ETH_RMII_REF_CLK------------------> PA1
        ETH_RMII_CRS_DV ------------------> PA7
        ETH_RMII_RXD0 --------------------> PC4
        ETH_RMII_RXD1 --------------------> PC5
        ETH_RMII_TX_EN -------------------> PG11
        ETH_RMII_TXD0 --------------------> PG13
        ETH_RMII_TXD1 --------------------> PG14
        ETH_RESET-------------------------> PD3
 
 ***************************************************************************************************
 * ÊµÑéÏÖÏó
 * 1 Ê×ÏÈ»ñÈ¡DHCP·ÖÅäµÄIPÐÅÏ¢£¬ÈôDHCP·ÖÅäÊ§°Ü£¬ÔòÏµÍ³Ê¹ÓÃ¾²Ì¬µÄIPÐÅÏ¢£¨ÇëÔÚlwip_comm.cÎÄ¼þÏÂµÄlwip_comm_default_ip_setº¯ÊýÖÐÐÞ¸Ä£©¡£
 * 2 °´ÏÂ¼üÅÌ¡°WIN+R¡±¿ì½Ý¼ü²¢ÊäÈë¡°CMD¡±ÃüÁî½øÈëµçÄÔµÄÃüÁî½çÃæ£¬ÔÚ´Ë½çÃæÏÂÊäÈë¡°ping xxxxx¡±Ö¸Áî£¬ÈôÄÜpingÍ¨£¬Ôò¸Ã¹¤³ÌÒÆÖ²³É¹¦¡£

 ***************************************************************************************************
 * ×¢ÒâÊÂÏî
 * 1 µçÄÔ¶Ë´®¿Úµ÷ÊÔÖúÊÖ²¨ÌØÂÊ±ØÐëÊÇ115200
 * 2 ÇëÊ¹ÓÃXCOM/SSCOM´®¿Úµ÷ÊÔÖúÊÖ,ÆäËû´®¿ÚÖúÊÖ¿ÉÄÜ¿ØÖÆDTR/RTSµ¼ÖÂMCU¸´Î»/³ÌÐò²»ÔËÐÐ
 * 3 ´®¿ÚÊäÈë×Ö·û´®ÒÔ»Ø³µ»»ÐÐ½áÊø
 * 4 ÇëÓÃUSBÏßÁ¬½ÓÔÚUSB_UART,ÕÒµ½USB×ª´®¿Úºó²âÊÔ±¾Àý³Ì
 * 5 P4µÄPA9/PA10±ØÐëÍ¨¹ýÌøÏßÃ±Á¬½ÓÔÚRXD/TXDÉÏ
 * 6 ±¾Àý³Ì½öÖ§³ÖMCUÆÁ£¬²»Ö§³ÖRGBÆÁ
 * 7 4.3´çºÍ7´çÆÁÐèÒª±È½Ï´óµçÁ÷,USB¹©µç¿ÉÄÜ²»×ã,ÇëÓÃÍâ²¿µçÔ´ÊÊÅäÆ÷(ÍÆ¼öÍâ½Ó12V 1AµçÔ´).
 * 8 ÇÐ¼ÇÐ´ÈëµØÖ·²»ÄÜÊÇÓÃ»§´úÂëÇø£¬·ñÔò¿ÉÄÜËÀ»ú

 ***************************************************************************************************
 * ¹«Ë¾Ãû³Æ£º¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾£¨ÕýµãÔ­×Ó£©
 * µç»°ºÅÂë£º020-38271790
 * ´«ÕæºÅÂë£º020-36773971
 * ¹«Ë¾ÍøÖ·£ºwww.alientek.com
 * ¹ºÂòµØÖ·£ºzhengdianyuanzi.tmall.com
 * ¼¼ÊõÂÛÌ³£ºhttp://www.openedv.com/forum.php
 * ×îÐÂ×ÊÁÏ£ºwww.openedv.com/docs/index.html
 *
 * ÔÚÏßÊÓÆµ£ºwww.yuanzige.com
 * B Õ¾ÊÓÆµ£ºspace.bilibili.com/394620890
 * ¹« ÖÚ ºÅ£ºmp.weixin.qq.com/s/y--mG3qQT8gop0VRuER9bw
 * ¶¶    Òô£ºdouyin.com/user/MS4wLjABAAAAi5E95JUBpqsW5kgMEaagtIITIl15hAJvMO8vQMV1tT6PEsw-V5HbkNLlLMkFf1Bd
 ***************************************************************************************************
 */
