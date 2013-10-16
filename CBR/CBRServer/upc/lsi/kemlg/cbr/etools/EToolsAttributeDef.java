package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import java.util.*;

/**
 * This class adapts the AttributeDef class in order to be used in the A-TEAM Project. <br>
 * This class adds a method to parse part of the RTXPS Descriptor files, where all the
 * parameters of the A-TEAM project are defined.
 * @see AttributeDef
 * @see EToolsCBR
 * @see EToolsCaseDescription
 */
public class EToolsAttributeDef extends AttributeDef
                               implements java.io.Serializable
{   
    
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates an EToolsAttributeDef with the given name, type and weight. <br> 
   * <br>
   * {PRE: name is a string suitable (informative) for naming the attribute definition,
   *       type is an int with value AttributeDef.CATEGORICAL or 
   *            AttributeDef.LINEAR, 
   *       weight is the weight of the attribute
   * }<br>
   * {POST: an instance of EToolsAttributeDef has been created.
   * }<br>
   * <br>
   * @param name the name of the attribute definition
   * @param type the type of the attribute (CATEGORICAL or LINEAR)
   * @param weight the weight of the attribute. 
   */
    public EToolsAttributeDef(String name, int type, float weight)
    {
       super(name, type, weight);
    }
    
    
  /**
   * Creates an EToolsAttributeDef with the given name, type, weight and order value. <br> 
   * <br>
   * {PRE: name is a string suitable (informative) for naming the attribute definition,
   *       type is an int with value AttributeDef.CATEGORICAL or 
   *            AttributeDef.LINEAR, 
   *       weight is the weight of the attribute
   *       order is a boolean which tells whether the attribute modalities have an order or not.
   * }<br>
   * {POST: an instance of EToolsAttributeDef has been created.
   * }<br>
   * <br>
   * @param name the name of the attribute definition
   * @param type the type of the attribute (CATEGORICAL or LINEAR)
   * @param weight the weight of the attribute. 
   * @param order the attribute modalities have an order
   */
    public EToolsAttributeDef(String name, int type, float weight,
                             boolean order)
    {
       super(name, type, weight, order);
    } 
    
    
  /**
   * Parses part of the RTXPS Descriptor files, getting the definition of an attribute. <br>
   * No instance of EToolsAttributeDef is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the RTXPS descriptors
   * }<br>
   * {POST: returns an instance of EToolsAttributeDef with the attribute definition parsed from in.
   * }<br>
   * <br>
   * @param in the source of attribute definitions to be parsed 
   * @return EToolsAttributeDef the attribute definition obtained.
   */
    public static EToolsAttributeDef importFromRTXPS (java.io.BufferedReader in)
                                    throws java.io.IOException 
    {
        String line = "";
        boolean end_of_file = false;
        
        System.out.println("atrdef: he entrado");
        //variables auxiliares de la definición del atributo 
        String nameAtr, typeStr, lineStr;
        int type = -1; 
        float weight = 1.0f; //no tenemos pesos en el fichero
        EToolsAttributeDef res;
          
        //variables auxiliares para la modalidad
        Vector modalitiesFile;
        String modStr = "";  
          
        //1.buscar el inicio del descriptor
        System.out.println("atrdef: busco DESCRIPTOR");
        while ((!end_of_file)&&(!line.startsWith("DESCRIPTOR")))
        {
          line = in.readLine();
          System.out.println(line);
          if (line == null)
          {  
             end_of_file = true;
          }
          else
          {    
             line = line.trim();
          }
        }        
        
        System.out.println("atrdef: he encontrado DESCRIPTOR o EOF");
        if (end_of_file)
        {
           System.out.println("atrdef: encontré EOF");
           res = null;
        }
        else
        {  System.out.println("atrdef: encontré DESCRIPTOR");  
           //2.leer el nombre de la variable
           nameAtr = in.readLine();
           nameAtr = nameAtr.trim();
           if (nameAtr.endsWith(":"))
           {
              nameAtr = nameAtr.substring(0,(nameAtr.length() - 1));
           }
         
           System.out.println("atrdef: he leido nameAtr = " + nameAtr);         
           
           System.out.println("atrdef: busco la linea del tipo");
           System.out.println(line);
           //3.buscar el tipo de la variable
           while (!line.startsWith("T"))
           {
             line = in.readLine();
             line = line.trim();
             System.out.println(line);
           } 
           
           System.out.println("atrdef: he encontrado el tipo");
           //4.obtener el tipo de la variable
           lineStr = line.substring(2);
           line = in.readLine();
           line = line.trim(); //la linea siguiente
           System.out.println("atrdef:tipo=" +  lineStr + "\n linea siguiente=" + line);  
                      
           if ((lineStr.startsWith("S")) && (line.startsWith("U")))
           {  //si es de tipo simbólico para RTXPS y tiene unidades
              //asociadas... es lo que nosotros llamamos LINEAR
              
              System.out.println("atrdef:T=S y U");
              type = AttributeDef.LINEAR;
              line = in.readLine();
              line = line.trim(); //saltamos al inicio de 
                                  //las modalidades              
              
           }
           else if ((lineStr.startsWith("S")) && (line.startsWith("V")))
           {  //si es de tipo simbólico para RTXPS y no tiene unidades
              //asociadas... es lo que nosotros llamamos CATEGORICAL

              System.out.println("atrdef:T=S y V");
              type = AttributeDef.CATEGORICAL;
              
              //Nota: ya estamos en el inicio de las modalidades
           }
           //else -> no tengo más ejemplos por ahora de tipos
         
           //5.creamos una instancia de EToolsAttributeDef, incompleta
           res = new EToolsAttributeDef(nameAtr, type, weight);
        
           //6.buscar el inicio de las modalidades
         
           System.out.println("atrdef: estamos al inicio de las modalidades /n" + line);
           //while (!line.startsWith("V"))
           //{
           //  line = in.readLine();
           //  line = line.trim();
           //} 
         
           System.out.println("atrdef: leemos las modalidades");
           //7.leer las modalidades y ponerlas en un String
           while (line.startsWith("V"))
           {
              System.out.println(line);
              modStr += line + "\n";
              line = in.readLine();
              line = line.trim();
           }
         
           modStr += "\n *** ***";
         
           //8.crear un vector de modalidades a partir de lo leido
           //  e insertarlo en la definición del atributo
           System.out.println("atrdef: voy a llamar a atrmod");
           EToolsModality.importFromRTXPS(modStr,res);
           System.out.println("atrdef: he vuelto de atrmod");
           
           //9.saltarse el resto del descriptor 
           while (!line.equals("ENDDESCRIPTOR"))
           {   
              line = in.readLine();
              line = line.trim();
           }
        }
        
        return res;
    }  


  /**
   * Parses part of the RTXPS Descriptor files, getting the definition of an attribute. <br>
   * No instance of EToolsAttributeDef is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the RTXPS descriptors
   * }<br>
   * {POST: returns an instance of EToolsAttributeDef with the attribute definition parsed from in.
   * }<br>
   * <br>
   * @param in the source of attribute definitions to be parsed 
   * @return EToolsAttributeDef the attribute definition obtained.
   */
    public static EToolsAttributeDef importFromRTXPS (java.io.BufferedReader in,boolean debug)
                                    throws java.io.IOException 
    {
        String line = "";
        boolean end_of_file = false;
        
        if (debug)
           System.out.println("atrdef: he entrado");
        //variables auxiliares de la definición del atributo 
        String nameAtr, typeStr, lineStr;
        int type = -1; 
        float weight = 1.0f; //no tenemos pesos en el fichero
        EToolsAttributeDef res;
          
        //variables auxiliares para la modalidad
        Vector modalitiesFile;
        String modStr = "";  
          
        //1.buscar el inicio del descriptor
        if (debug)
           System.out.println("atrdef: busco DESCRIPTOR");
        while ((!end_of_file)&&(!line.startsWith("DESCRIPTOR")))
        {
          line = in.readLine();
          if (debug)
          	 System.out.println(line);
          if (line == null)
          {  
             end_of_file = true;
          }
          else
          {    
             line = line.trim();
          }
        }        
        
        if (debug)
           System.out.println("atrdef: he encontrado DESCRIPTOR o EOF");
        if (end_of_file)
        {
	        if (debug)
              System.out.println("atrdef: encontré EOF");
           res = null;
        }
        else
        {  
	        if (debug)
	           System.out.println("atrdef: encontré DESCRIPTOR");  
           //2.leer el nombre de la variable
           nameAtr = in.readLine();
           nameAtr = nameAtr.trim();
           if (nameAtr.endsWith(":"))
           {
              nameAtr = nameAtr.substring(0,(nameAtr.length() - 1));
           }
         
           if (debug)
           {
              System.out.println("atrdef: he leido nameAtr = " + nameAtr);         
              System.out.println("atrdef: busco la linea del tipo");
              System.out.println(line);
           }
           //3.buscar el tipo de la variable
           while (!line.startsWith("T"))
           {
             line = in.readLine();
             line = line.trim();
             if (debug)
                System.out.println(line);
           } 
           
           if (debug)
              System.out.println("atrdef: he encontrado el tipo");
           //4.obtener el tipo de la variable
           lineStr = line.substring(2);
           line = in.readLine();
           line = line.trim(); //la linea siguiente
           if (debug)
              System.out.println("atrdef:tipo=" +  lineStr + "\n linea siguiente=" + line);  
                      
           if ((lineStr.startsWith("S")) && (line.startsWith("U")))
           {  //si es de tipo simbólico para RTXPS y tiene unidades
              //asociadas... es lo que nosotros llamamos LINEAR
              if (debug)
                 System.out.println("atrdef:T=S y U");
              type = AttributeDef.LINEAR;
              line = in.readLine();
              line = line.trim(); //saltamos al inicio de 
                                  //las modalidades              
              
           }
           else if ((lineStr.startsWith("S")) && (line.startsWith("V")))
           {  //si es de tipo simbólico para RTXPS y no tiene unidades
              //asociadas... es lo que nosotros llamamos CATEGORICAL
				  if (debug)
                 System.out.println("atrdef:T=S y V");
              type = AttributeDef.CATEGORICAL;
              
              //Nota: ya estamos en el inicio de las modalidades
           }
           //else -> no tengo más ejemplos por ahora de tipos
         
           //5.creamos una instancia de EToolsAttributeDef, incompleta
           res = new EToolsAttributeDef(nameAtr, type, weight);
        
           //6.buscar el inicio de las modalidades
           if (debug)
              System.out.println("atrdef: estamos al inicio de las modalidades /n" + line);
           //while (!line.startsWith("V"))
           //{
           //  line = in.readLine();
           //  line = line.trim();
           //} 
           if (debug)
              System.out.println("atrdef: leemos las modalidades");
           //7.leer las modalidades y ponerlas en un String
           while (line.startsWith("V"))
           {
	           if (debug)
                 System.out.println(line);
              modStr += line + "\n";
              line = in.readLine();
              line = line.trim();
           }
         
           modStr += "\n *** ***";
         
           //8.crear un vector de modalidades a partir de lo leido
           //  e insertarlo en la definición del atributo
           if (debug)
              System.out.println("atrdef: voy a llamar a atrmod");
           EToolsModality.importFromRTXPS(modStr,res);
           if (debug)
              System.out.println("atrdef: he vuelto de atrmod");
           
           //9.saltarse el resto del descriptor 
           while (!line.equals("ENDDESCRIPTOR"))
           {   
              line = in.readLine();
              line = line.trim();
           }
        }
        
        return res;
    }  

            
}