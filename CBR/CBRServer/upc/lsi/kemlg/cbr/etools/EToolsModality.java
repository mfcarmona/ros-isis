package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import java.util.*;

/**
 * This class adapts the Modality class in order to be used in the A-TEAM Project. <br>
 * This class adds a method to parse the part of the RTXPS Descriptor files which
 * defines modalities of attributes.
 * @see Modality
 * @see EToolsAttributeTable
 * @see EToolsAttributeDef
 */
public class EToolsModality extends Modality 
                           implements java.io.Serializable
{     

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates a modality which belongs to the given attribute definition, with the given numeric identifier, qualitative identifier and bottom and top values. <br> 
   * <br>
   * <b>Note: this constructor is only suitable for linear attributes</b>.
   * {PRE: atr is an instance of AttributeDef,
   *       numMod is an integer value which numerically identifies the modality,
   *       qualMod is an String value which gives a qualitative value to the modality,
   *       bottomVal is a float value,
   *       topVal is a float value and bottomVal is lower than topVal.
   * }<br>
   * {POST: an instance of EToolsModality has been created.
   * }<br>
   */
    public EToolsModality (AttributeDef atr, int numMod, String qualMod, 
              float bottomVal, float topVal)
    {
       super(atr, numMod, qualMod, bottomVal, topVal);
    } 
    

  /**
   * Creates a modality which belongs to the given attribute definition, with the given numeric identifier, qualitative identifier. <br> 
   * <br>
   * <b>Note: this constructor is only suitable for categorical attributes</b>.
   * {PRE: atr is an instance of AttributeDef,
   *       numMod is an integer value which numerically identifies the modality,
   *       qualMod is an String value which gives a qualitative value to the modality,
   * }<br>
   * {POST: an instance of EToolsModality has been created.
   * }<br>
   */
    public EToolsModality (AttributeDef atr, int numMod, String qualMod)
    {
       super(atr, numMod, qualMod);      
    } 
    

  /**
   * Parses part of the RTXPS Descriptor files, getting the definition of the modalities
   * and adding them to the give attribute definition. <br>
   * No instance of EToolsAttributeDef is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the RTXPS descriptors,
   *       atrDef is an instance of AttributeDef
   * }<br>
   * {POST: atrDef has been filled with instances of EToolsModality, each one a modality parsed from in.
   * }<br>
   * <br>
   * @param in the source of attribute definitions to be parsed 
   * @param atrDef the attribute definition where the parsed modalities should be added.
   */
    public static void importFromRTXPS (java.lang.String in, AttributeDef atrDef)
    {
       StringTokenizer st = new StringTokenizer(in," ,/[]\n");
       String token = "";
       boolean haLeidoV = false;
       boolean haLeidoQualMod = false;
       
       EToolsModality modAct;
       String qualMod;
       int numMod;
       int contMod = 0; //contador del número de modalidades
                        //útil solo para atributos LINEAR
       float bottomVal, topVal, auxVal;

             
       while ((st.hasMoreTokens()) && (!token.equals("***")))
       {
       
          //1. saltamos la V inicial
          if (!haLeidoV)
          {  
             token = st.nextToken();
             //System.out.println("[" + token + "]");
          }
          else
          {
             haLeidoV = false;
          }  
          
          //2. obtenemos el nombre cualitativo de la modalidad
          if (!haLeidoQualMod)
          {  
             qualMod = st.nextToken();
             //System.out.println("<" + qualMod + ">qualMod");
          }
          else
          {
             qualMod = token;
             haLeidoQualMod = false;
          }
          
          if (atrDef.getAttrType()==AttributeDef.LINEAR)
          {
             //3a. obtenemos el bottomVal
             token = st.nextToken();
             //System.out.println("<" + token + ">parse1");
             bottomVal = Float.parseFloat(token); 
            
             //4a. obtenemos el valor del medio (topVal)
             token = st.nextToken();
             //System.out.println("<" + token + ">parse2");
             topVal = Float.parseFloat(token); 
            
             //5a. obtenemos el siguente valor (si lo hay),
             //    y se calcula el numero de modalidad por el
             //    contador de modalidades
             if (st.hasMoreTokens())
             {  
                token = st.nextToken();
                //System.out.println("¿" + token + "?parse3");
                if ((!token.equals("V"))&&(!token.startsWith("*")))
                {
                   // tenemos un valor más, que es realmente el topVal
                   try
                   {
                     auxVal = Float.parseFloat(token);
                     topVal = auxVal; 
                     token = st.nextToken();
                     //System.out.println("[" + token + "]");
                     haLeidoV = true;
                   }
                   catch (NumberFormatException e)
                   {
                      haLeidoV = true;
                      haLeidoQualMod = true;
                   } 
                     
                }
                else
                { 
                   haLeidoV = true; //  -> ya estamos en el siguiente valor
                   haLeidoQualMod = false; 
                }
             }     
             numMod = contMod;
            
             //6a. creamos la modalidad que acabamos de parsear
             //    y la añadimos a la definición del atributo
             modAct = new EToolsModality(atrDef, numMod, qualMod,
                                       bottomVal, topVal);
             atrDef.addModality(modAct);
             //System.out.println(modAct);
          }
          else //atributo CATEGORICAL
          {
             //3b. saltamos este valor
             token = st.nextToken();
             //System.out.println("[" + token + "]");
             
             //4b. saltamos este valor
             token = st.nextToken();
             //System.out.println("[" + token + "]");
             
             //5b. obtenemos el numero de modalidad
             token = st.nextToken();
             //System.out.println("<" + token + ">");
             numMod = (Integer.parseInt(token) - 1);
             // ********************* WARNING! ******************
             // esto es un parche provisional, porque la clase Vector
             // no puede ir de las posiciones 1 a N!!!!! habria que
             // arreglar AttributeDef!!!! 
            
             //6b. creamos la modalidad que acabamos de parsear
             //    y la añadimos a la definición del atributo
             modAct = new EToolsModality(atrDef, numMod, qualMod);
             atrDef.addModality(modAct);
             token = st.nextToken();
             //System.out.println("[" + token + "]");
             haLeidoV = true;
          }  
          contMod++;                             
       }

    }
 
}