package upc.lsi.kemlg.cbr.etools;

import java.util.*;
import upc.lsi.kemlg.cbr.*;

/**
 * This class adapts the AttributeTable class in order to be used in the A-TEAM Project. <br>
 * This class adds a method to parse the RTXPS Descriptor files, where all the
 * parameters of the A-TEAM project are defined.
 * @see AttributeTable
 * @see EToolsCBR
 * @see EToolsCaseDescription
 */
public class EToolsAttributeTable extends AttributeTable
                                 implements java.io.Serializable
{   
  
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates a void EToolsAttributeTable. <br> 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of EToolsAttributeTable has been created.
   * }<br>
   */  
    public EToolsAttributeTable()
    {
       super();
    }
    

  /**
   * Parses the RTXPS Descriptor files, getting the definition of each attribute and
   * creating an EToolsAttributeTable containing all the parsed attribute definitions. <br>
   * No instance of EToolsAttributeTable is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the RTXPS descriptors
   * }<br>
   * {POST: returns an instance of EToolsAttributeTable with all the attribute definitions parsed from in.
   * }<br>
   * <br>
   * @param in the source of attribute definitions to be parsed 
   * @return EToolsAttributeTable the attribute table with all the attribute definitions obtained.
   */
    public static EToolsAttributeTable importFromRTXPS (java.io.BufferedReader in)
    {
       boolean end_of_file = false;
       EToolsAttributeTable attrTab;
       EToolsAttributeDef aux=null;
     
       System.out.println("atrtab: he entrado");  
       attrTab = new EToolsAttributeTable();
       System.out.println("atrtab: voy a leer los atributos");  
       try
       {  
         while (!end_of_file) 
         {
            System.out.println("atrtab: no he llegado a EOF");  
            System.out.println("atrtab: voy a llamar a atrdef");
            aux = EToolsAttributeDef.importFromRTXPS(in);
            System.out.println("atrtab: he vuelto de atrdef");
            if (aux == null)
            {
               System.out.println("atrtab: no he leido nada, estoy en EOF");
               end_of_file = true;
            }
            else
            {
               System.out.println("atrtab: he leido el atributo\n" + aux);    
               attrTab.add(aux);
            }
         }
       }
       catch (java.io.IOException e)
       {
         e.printStackTrace();
       }   
       
       return attrTab;  
    }
    
    /**
   * Parses the RTXPS Descriptor files, getting the definition of each attribute and
   * creating an EToolsAttributeTable containing all the parsed attribute definitions. <br>
   * No instance of EToolsAttributeTable is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the RTXPS descriptors
   * }<br>
   * {POST: returns an instance of EToolsAttributeTable with all the attribute definitions parsed from in.
   * }<br>
   * <br>
   * @param in the source of attribute definitions to be parsed 
   * @return EToolsAttributeTable the attribute table with all the attribute definitions obtained.
   */
    public static EToolsAttributeTable importFromRTXPS (java.io.BufferedReader in,boolean debug)
    {
       boolean end_of_file = false;
       EToolsAttributeTable attrTab;
       EToolsAttributeDef aux=null;
       
       if (debug)
       	System.out.println("atrtab: he entrado");  
       attrTab = new EToolsAttributeTable();
       if (debug)
       	System.out.println("atrtab: voy a leer los atributos");  
       try
       {  
         while (!end_of_file) 
         {
	         if (debug)
	         {
            	System.out.println("atrtab: no he llegado a EOF");  
            	System.out.println("atrtab: voy a llamar a atrdef");
         	}
            aux = EToolsAttributeDef.importFromRTXPS(in,debug);
            if (debug)
            	System.out.println("atrtab: he vuelto de atrdef");
            if (aux == null)
            {
	            if (debug)
               	System.out.println("atrtab: no he leido nada, estoy en EOF");
               end_of_file = true;
            }
            else
            {
	            if (debug)
               	System.out.println("atrtab: he leido el atributo\n" + aux);    
               attrTab.add(aux);
            }
         }
       }
       catch (java.io.IOException e)
       {
         e.printStackTrace();
       }   
       
       return attrTab;  
    }

}