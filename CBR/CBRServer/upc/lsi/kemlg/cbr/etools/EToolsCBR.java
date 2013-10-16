package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.general.*;
import upc.lsi.kemlg.cbr.etools.*;
import upc.lsi.kemlg.cbr.distances.*;
import java.io.*;


/**
 * This class adapts the CBR to be used in the A-TEAM Project. <br>
 * In this class the behaviour of the CBR (its reasoning cycle) is 
 * totally defined. A flat memory structure has been chosen, to
 * always find the best match to a given query. Attribute-value
 * pairs have been chosen as case descriptors. <br>
 * It also adds methods to parse the simulation data files the
 * CBR server receives off-line.
 * Note: to parse the RTXPS Descriptor files it should be used the
 * method importFromRTXPS from class EToolsAttributeTable.
 * @see CBR
 * @see PlainCBR
 * @see Case
 * @see AttributeValList
 * @see EToolsAttributeTable
 */
public class EToolsCBR extends PlainCBR implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** 
   *  AttributeVal Definition Table, that defines all the attributes 
   *  needed to describe a case.  
   */ 
   private AttributeTable atrTab;
   
  /** 
   *  Distance Measure used to compare cases. 
   */
   private ComposedDistance distMeasure;
  

  
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   
 /**
   * Creates an empty EToolsCBR with a void name. <br> 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of EToolsCBR has been created.
   * }<br>
   */
   public EToolsCBR()
   {
      super();
   }
   

  /**
   * Creates an empty EToolsCBR with the given name. <br> 
   * <br>
   * {PRE: nameCBR is a string suitable (informative) for naming the 
   *       Case Library,
   * }<br>
   * {POST: an instance of EToolsCBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   */
   public EToolsCBR(String nameCBR)
   {
      super(nameCBR);
   }
   

  /**
   * Creates an empty EToolsCBR with a void name and sets genIds as the Case Identifiers' generator. <br> 
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br>
   * <br>
   * {PRE: genIds is an instance of IdGenerator.
   * }<br>
   * {POST: an instance of EToolsCBR has been created.
   * }<br>
   * @param genIds  the automatic Case Identifiers' generator that will be
   *                used.
   */
   public EToolsCBR(IdGenerator genIds)
   {
      super(genIds);
   }
     

  /**
   * Creates an empty EToolsCBR with the given name and sets genIds as the Case Identifiers' generator. <br>
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br> 
   * <br>
   * {PRE: nameCBR is a string suitable (informative) for naming the Case Lib,
   *       genIds is an Identifier generator.
   * }<br>
   * {POST: an instance of EToolsCBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   * @param genIds the automatic Case Identifiers' generator that will be
   *                used.
   */
   public EToolsCBR(String nameCBR, IdGenerator genIds)
   {
      super(nameCBR,genIds);
   }
     
   
   /**
   * Gets the Attribute Table used by the Case Lib. <br>
   * It gives access to the internal Attribute Table, but it should only
   * be used to consult attribute definitions. It's very dangerous to add
   * or delete attributes at the AttributeVal list when cases have been added
   * at the Case Lib.<br> 
   * <br>
   * {PRE: x is an instance of EToolsCBR
   * }<br>
   * {POST: returns the attribute table used by x.
   * }<br>
   * <br>
   * @return AttributeTable the attribute table used by the Case Library 
   */
   public AttributeTable getAttributeTable()
   {
      if (atrTab == null)
      {
         throw new java.lang.RuntimeException("You must set an AttributeTable before. Use the setAttributeTable method");
      }
      return atrTab;
   }
   
   
  /**
   * Sets the Attribute Table to be used by the Case Lib. <br>
   * It should only be used when there are no cases introduced at the
   * Case Library.<br> 
   * <br>
   * {PRE: x is an instance of EToolsCBR and
           x is empty of cases,
   *       atrTab is an instance of AttributeTable
   * }<br>
   * {POST: atrTab is the Attribute Table used by x.
   * }<br>
   * <br>
   * @param atrTab the attribute table to be used by the Case Library 
   */
   public void setAttributeTable(AttributeTable atrTab)
   {
      this.atrTab = atrTab;
   }
 
   
 /**
   * Gets the Distance Measure used by the Case Lib. <br>
   * It gives access to the internal Distance Measure.<br> 
   * <br>
   * {PRE: x is an instance of EToolsCBR
   * }<br>
   * {POST: returns the distance measure used by x.
   * }<br>
   * <br>
   * @return ComposedDistance the distance measure used by the Case Library 
   */
   public ComposedDistance getDistanceMeasure()
   {
      if (distMeasure == null)
      {
         throw new java.lang.RuntimeException("You must set an ComposedDistance before. Use the setDistanceMeasure method");
      }
      return distMeasure;
   }
   
   
  /**
   * Sets the Distance Measure to be used by the Case Lib. <br>
   * This method should only be used when there are no cases introduced at
   * the Case Library.<br> 
   * <br>
   * {PRE: x is an instance of EToolsCBR and
   *       x is empty of cases,
   *       distMeasure is an instance of ComposedDistance
   * }<br>
   * {POST: distMeasure is DistanceMeasure used by x.
   * }<br>
   * <br>
   * @param distMeasure the DistanceMeasure to be used by the Case Library 
   */
   public void setDistanceMeasure(ComposedDistance distMeasure)
   {
      this.distMeasure = distMeasure;
   }
   

  /**
   * Parses the off-line simulation data, adding the information as cases in the Case Lib. <br>
   * {PRE: x is an instance of EToolsCBR and
   *       in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: all the cases parsed from in have been added to x.
   * }<br>
   * <br>
   * @param in the source of cases to be parsed 
   */
   public void importFromData(java.io.BufferedReader in)
   {  
   	  java.util.Vector vectorVars;
      String[] arrayDesc; //para guardar los nombres de vars de INPUTP
      int numVarsDesc;
	  String[] arrayOut;  //para guardar los nombres de vars de OUTPUTP
	  int numVarsOut;
      float[] arrayVal;
      int arrayPos = 0;
      String line;
	  String nombreCBR;
	  String aux;
      /*StreamTokenizer st;
      int tokenType;
      float tokenNum =-1.0f;
      String tokenStr = "";
      boolean concatenaSiguiente = false;*/
      EToolsCase auxCase;
      int contCases = 0;
      
      
      try
      {
	     //1. Leemos el HEAD
	       
         line = in.readLine();
		 
		 //1.1 Obtenemos el nombre de la Libreria de casos
		 
         while ((line != null) && (!line.startsWith("CI ")))
 	     {
		 	line = in.readLine();
 	     }
		 if (line == null)
		 {
		    throw new java.io.IOException("INCORRECT format of the data. CI not found.");
		 }
		  
	     //System.out.println("EToolsCBR: Encontrado CI");
		 System.out.println(line);
		 
		 line.trim();
		 nombreCBR = line.substring(3);
		 line = in.readLine();
		 if (line.startsWith("ET "))
		 {
		    //System.out.println("EToolsCBR: Tenemos ET");
		    line.trim();
		    nombreCBR = nombreCBR + "_" + line.substring(3);
		 }
		 System.out.println("EToolsCBR: nombreCBR = " + nombreCBR);
		 setNameCBR(nombreCBR);
						
		 //1.2 leemos los nombres y el orden de las variables INPUTP
		 
		 //avanzamos hasta el inicio de las variables INPUTP
		 //System.out.println("EToolsCBR: Buscando INPUTP_ORDER");

		 while ((line != null) && (!line.startsWith("INPUTP_ORDER")))
		 {
  		     //System.out.println("line = " + line);
			
		     line = in.readLine();			
		 }
		 if (line == null)
		 {
            throw new java.io.IOException("INCORRECT format of the data. INPUTP_ORDER not found.");
         } 
						
		 //System.out.println("EToolsCBR: INPUTP_ORDER encontrado.");

		 //leemos los nombres de variables y guardamos el orden
 	     line = in.readLine();
         vectorVars = new java.util.Vector();
		 numVarsDesc = 0;
		 System.out.println("EToolsCBR: leyendo nombres variables");
			   
         while(line.startsWith("P "))
         { 
            vectorVars.add(line.substring(2).trim());
            numVarsDesc ++;
            line = in.readLine();
         }
         System.out.println("INPUTP = " + vectorVars);
                
         arrayDesc = new String[numVarsDesc];
               
		 vectorVars.copyInto(arrayDesc);
		 //System.out.println("EToolsCBR: nombres de variables INPUTP leidos");
			   

         //1.3 Se obtiene la información sobre los TRAINEE
		 
		    //WARNING: aqui habria que tratar los TRAINEE


         //1.4 leemos los nombres y el orden de las variables OUTPUTP

         //avanzamos hasta el inicio de las variables OUTPUTP
         //System.out.println("EToolsCBR: Buscando OUTPUTP_ORDER");

         while ((line != null) && (!line.startsWith("OUTPUTP_ORDER")))
         {
            //System.out.println("line = " + line);
            
            line = in.readLine();			
         }
         if (line == null)
         {
            throw new java.io.IOException("INCORRECT format of the data. OUTPUTP_ORDER not found.");
         } 
         
         System.out.println("EToolsCBR: OUTPUTP_ORDER encontrado.");

         //leemos los nombres de variables y guardamos el orden
         line = in.readLine();
         vectorVars.clear();
         numVarsOut = 0;
         //System.out.println("EToolsCBR: leyendo nombres variables");
               
         while(line.startsWith("P "))
         { 
            vectorVars.add(line.substring(2).trim());
            numVarsOut ++;
            line = in.readLine();
         }
		 
         //System.out.println("OUTPUTP = " + vectorVars);
                
         arrayOut = new String[numVarsOut];
               
         vectorVars.copyInto(arrayOut);
         //System.out.println("EToolsCBR: nombres de variables OUTPUTP leidos");
      


         //System.out.println("EToolsCBR: Saltando al final de HEAD");

		 while ((line != null) && (!line.startsWith("ENDHEAD")))
		 {
			line = in.readLine();			
		 }
		 if (line == null) //se ha llegado prematuramente a final de fichero
		 {
		    throw new java.io.IOException("INCORRECT format of the data. ENDHEAD not found.");
		 }
		 //else estamos en el final de HEAD
 
		 
         //2. Comprobamos si las variables estan todas en la tabla de
         //   atributos (las variables que no estén serán ignoradas)

         System.out.println("EToolsCBR: vamos a mostrar el array de variables");
         
         aux = "INPUTP = #";
         for (int i=0;i<numVarsDesc;i++)
         {
           aux += arrayDesc[i] + "#";
         }
         System.out.println(aux);

         aux = "OUTPUTP = #";
         for (int i=0;i<numVarsOut;i++)
         {
           aux += arrayOut[i] + "#";
         }
         System.out.println(aux);
         

         System.out.println("EToolsCBR: comprobacion de variables");

         AttributeTable atrTab = getAttributeTable();
         for (int i=0; i<numVarsDesc; i++)
         {
            if (!atrTab.containsKey(arrayDesc[i]))
            {
               System.out.println("WARNING: attribute table does NOT contain the attribute " + arrayDesc[i] + ". It will be skipped");
            }
         }

         for (int i=0; i<numVarsOut; i++)
         {
            if (!atrTab.containsKey(arrayOut[i]))
            {
               System.out.println("WARNING: attribute table does NOT contain the attribute " + arrayOut[i] + ". It will be skipped");
            }
         }
		 
          
         //3. leer los datos
		 
		 //buscamos BODY
		 
		 while ((line != null) && (!line.startsWith("BODY")))
         {
            //System.out.println("line = " + line);
            
            line = in.readLine();			
         }
         if (line == null)
         {
            throw new java.io.IOException("INCORRECT format of the data. BODY not found.");		 
         }
             
         while (line != null) //mientras no llegamos a EOF
         {
		    line = in.readLine();
		    //buscamos CASE o ENDBODY
		 
		    while ((line != null) && (!line.startsWith("CASE")) &&
		           (!line.startsWith("ENDBODY")))
			{
			
		       //System.out.println("line = " + line);
		    
		       line = in.readLine();			
		    }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. Error while looking for CASE or ENDBODY.");		 
		    }
			else if (line.startsWith("CASE"))
			{
		 	 
		       //pasamos el control a EToolsCase +  arrays vars
		 

		       //System.out.println("EToolsCBR: voy a llamar a EToolsCase");

               auxCase = EToolsCase.importFromData(getAttributeTable(),
                                                  getDistanceMeasure(),
                                                  arrayDesc,
				  								  numVarsDesc,
                                                  arrayOut,
                                                  numVarsOut,
                                                  in);      
                                                
               //System.out.println("EToolsCBR: he vuelto");
               //System.out.println(auxCase); 

               add(auxCase);
               contCases++;
			  
               if ((contCases%100)==0)
               {
                  System.out.println("" + contCases + " casos leidos...");
                  System.out.flush();
               }
			}
			else if (line.startsWith("ENDBODY"))
			{
				line = null;  //esto hará terminar el bucle
			}
         }
		  
       }
       catch (IOException e)
       {
          e.printStackTrace(); 
       }
       

     
   }   

   /**
   * Parses the off-line simulation data, adding the information as cases in the Case Lib. <br>
   * {PRE: x is an instance of EToolsCBR and
   *       in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: all the cases parsed from in have been added to x.
   * }<br>
   * <br>
   * @param in the source of cases to be parsed 
   */
   public void importFromData(java.io.BufferedReader in,boolean debug)
   {  
   	  java.util.Vector vectorVars;
      String[] arrayDesc; //para guardar los nombres de vars de INPUTP
      int numVarsDesc;
	  String[] arrayOut;  //para guardar los nombres de vars de OUTPUTP
	  int numVarsOut;
      float[] arrayVal;
      int arrayPos = 0;
      String line;
	  String nombreCBR;
	  String aux;
      /*StreamTokenizer st;
      int tokenType;
      float tokenNum =-1.0f;
      String tokenStr = "";
      boolean concatenaSiguiente = false;*/
      EToolsCase auxCase;
      int contCases = 0;
      
      
      try
      {
	     //1. Leemos el HEAD
	       
         line = in.readLine();
		 
		 //1.1 Obtenemos el nombre de la Libreria de casos
		 
         while ((line != null) && (!line.startsWith("CI ")))
 	     {
		 	line = in.readLine();
 	     }
		 if (line == null)
		 {
		    throw new java.io.IOException("INCORRECT format of the data. CI not found.");
		 }
		  
	     //System.out.println("EToolsCBR: Encontrado CI");
	    if (debug)
		    System.out.println(line);
		 
		 line.trim();
		 nombreCBR = line.substring(3);
		 line = in.readLine();
		 if (line.startsWith("ET "))
		 {
		    //System.out.println("EToolsCBR: Tenemos ET");
		    line.trim();
		    nombreCBR = nombreCBR + "_" + line.substring(3);
		 }
		 if (debug)
		    System.out.println("EToolsCBR: nombreCBR = " + nombreCBR);
		 setNameCBR(nombreCBR);
						
		 //1.2 leemos los nombres y el orden de las variables INPUTP
		 
		 //avanzamos hasta el inicio de las variables INPUTP
		 //System.out.println("EToolsCBR: Buscando INPUTP_ORDER");

		 while ((line != null) && (!line.startsWith("INPUTP_ORDER")))
		 {
  		     //System.out.println("line = " + line);
			
		     line = in.readLine();			
		 }
		 if (line == null)
		 {
            throw new java.io.IOException("INCORRECT format of the data. INPUTP_ORDER not found.");
         } 
						
		 //System.out.println("EToolsCBR: INPUTP_ORDER encontrado.");

		 //leemos los nombres de variables y guardamos el orden
 	     line = in.readLine();
         vectorVars = new java.util.Vector();
		 numVarsDesc = 0;
		 if (debug)
		    System.out.println("EToolsCBR: leyendo nombres variables");
			   
         while(line.startsWith("P "))
         { 
            vectorVars.add(line.substring(2).trim());
            numVarsDesc ++;
            line = in.readLine();
         }
         if (debug)
            System.out.println("INPUTP = " + vectorVars);
                
         arrayDesc = new String[numVarsDesc];
               
		 vectorVars.copyInto(arrayDesc);
		 //System.out.println("EToolsCBR: nombres de variables INPUTP leidos");
			   

         //1.3 Se obtiene la información sobre los TRAINEE
		 
		    //WARNING: aqui habria que tratar los TRAINEE


         //1.4 leemos los nombres y el orden de las variables OUTPUTP

         //avanzamos hasta el inicio de las variables OUTPUTP
         //System.out.println("EToolsCBR: Buscando OUTPUTP_ORDER");

         while ((line != null) && (!line.startsWith("OUTPUTP_ORDER")))
         {
            //System.out.println("line = " + line);
            
            line = in.readLine();			
         }
         if (line == null)
         {
            throw new java.io.IOException("INCORRECT format of the data. OUTPUTP_ORDER not found.");
         } 
         
         if (debug)
            System.out.println("EToolsCBR: OUTPUTP_ORDER encontrado.");

         //leemos los nombres de variables y guardamos el orden
         line = in.readLine();
         vectorVars.clear();
         numVarsOut = 0;
         //System.out.println("EToolsCBR: leyendo nombres variables");
               
         while(line.startsWith("P "))
         { 
            vectorVars.add(line.substring(2).trim());
            numVarsOut ++;
            line = in.readLine();
         }
		 
         //System.out.println("OUTPUTP = " + vectorVars);
                
         arrayOut = new String[numVarsOut];
               
         vectorVars.copyInto(arrayOut);
         //System.out.println("EToolsCBR: nombres de variables OUTPUTP leidos");
      


         //System.out.println("EToolsCBR: Saltando al final de HEAD");

		 while ((line != null) && (!line.startsWith("ENDHEAD")))
		 {
			line = in.readLine();			
		 }
		 if (line == null) //se ha llegado prematuramente a final de fichero
		 {
		    throw new java.io.IOException("INCORRECT format of the data. ENDHEAD not found.");
		 }
		 //else estamos en el final de HEAD
 
		 
         //2. Comprobamos si las variables estan todas en la tabla de
         //   atributos (las variables que no estén serán ignoradas)

         if (debug)
            System.out.println("EToolsCBR: vamos a mostrar el array de variables");
         
         aux = "INPUTP = #";
         for (int i=0;i<numVarsDesc;i++)
         {
           aux += arrayDesc[i] + "#";
         }
         if (debug)
            System.out.println(aux);

         aux = "OUTPUTP = #";
         for (int i=0;i<numVarsOut;i++)
         {
           aux += arrayOut[i] + "#";
         }
         if (debug)
         {
            System.out.println(aux);
            System.out.println("EToolsCBR: comprobacion de variables");
         }

         AttributeTable atrTab = getAttributeTable();
         for (int i=0; i<numVarsDesc; i++)
         {
            if (!atrTab.containsKey(arrayDesc[i]))
            {
	            // XXX if (debug)
               	System.out.println("WARNING: attribute table does NOT contain the attribute " + arrayDesc[i] + ". It will be skipped");
            }
         }

         for (int i=0; i<numVarsOut; i++)
         {
            if (!atrTab.containsKey(arrayOut[i]))
            {
	            // XXX if (debug)
               System.out.println("WARNING: attribute table does NOT contain the attribute " + arrayOut[i] + ". It will be skipped");
            }
         }
		 
          
         //3. leer los datos
		 
		 //buscamos BODY
		 
		 while ((line != null) && (!line.startsWith("BODY")))
         {
            //System.out.println("line = " + line);
            
            line = in.readLine();			
         }
         if (line == null)
         {
            throw new java.io.IOException("INCORRECT format of the data. BODY not found.");		 
         }
             
         while (line != null) //mientras no llegamos a EOF
         {
		    line = in.readLine();
		    //buscamos CASE o ENDBODY
		 
		    while ((line != null) && (!line.startsWith("CASE")) &&
		           (!line.startsWith("ENDBODY")))
			{
			
		       //System.out.println("line = " + line);
		    
		       line = in.readLine();			
		    }
		    if (line == null)
		    {
		       throw new java.io.IOException("INCORRECT format of the data. Error while looking for CASE or ENDBODY.");		 
		    }
			else if (line.startsWith("CASE"))
			{
		 	 
		       //pasamos el control a EToolsCase +  arrays vars
		 

		       //System.out.println("EToolsCBR: voy a llamar a EToolsCase");

               auxCase = EToolsCase.importFromData(getAttributeTable(),
                                                  getDistanceMeasure(),
                                                  arrayDesc,
				  								  numVarsDesc,
                                                  arrayOut,
                                                  numVarsOut,
                                                  in,debug);      
                                                
               //System.out.println("EToolsCBR: he vuelto");
               //System.out.println(auxCase); 

               add(auxCase);
               contCases++;
			  
               if ((contCases%100)==0)
               {
	               if (debug)
	               {
	                  System.out.println("" + contCases + " casos leidos...");
	                  System.out.flush();
               	}
               }
			}
			else if (line.startsWith("ENDBODY"))
			{
				line = null;  //esto hará terminar el bucle
			}
         }
		  
       }
       catch (IOException e)
       {
          e.printStackTrace(); 
       }
       

     
   } 

  /**
   * Parses the off-line simulation data, collecting it in a list of cases. <br>
   * No instance of EToolsCBR is needed to call this method, as it is an
   * static method.
   * {PRE: in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: returns a Java Vector with all the cases parsed from in.
   * }<br>
   * <br>
   * @param in the source of cases to be parsed 
   * @return java.util.Vector the vector of cases obtained.
   */
   public static java.util.Vector loadCurrentCases(java.io.BufferedReader in, 
                                                   AttributeTable atrTab,
                                                   ComposedDistance dist)
   {  
      String[] arrayDesc;
      float[] arrayVal;
      int arrayPos = 0;
      String line;
      StreamTokenizer st;
      int numVars;
      int tokenType;
      float tokenNum =-1.0f;
      String tokenStr = "";
      boolean concatenaSiguiente = false;
      EToolsCase auxCase;
      java.util.Vector caseVector =  new java.util.Vector();
      
      st =  new StreamTokenizer(in);
      st.eolIsSignificant(false); 
      
      try
      {  
         //leer la linea con el número de variables por caso
         st.nextToken();
         numVars = (int)st.nval;
      
         //System.out.println("numVars=" + numVars);
       
         //leer la linea de descripciones  
         arrayDesc = new String[numVars];
         while(st.nextToken() != StreamTokenizer.TT_NUMBER)
         {  
            if (st.ttype == StreamTokenizer.TT_WORD)
            {
               if (concatenaSiguiente)
               {  
                 tokenStr += st.sval;
                 arrayDesc[arrayPos]=tokenStr;
                 concatenaSiguiente = false;
                 arrayPos++;
               }  
               else
               {
                 tokenStr = st.sval;
                 arrayDesc[arrayPos]=tokenStr;
                 arrayPos++;
               }  
               //System.out.println("[" + tokenStr + "]");        
            }
            else if (st.ttype == '_')
            {
               tokenStr += "_";
               concatenaSiguiente = true;
               arrayPos--;
               //System.out.println("[" + tokenStr + "]");        
            }
            else if ((st.ttype == '<')||(st.ttype == '>'))
            {
               //nos los saltamos
            }
            else 
            {
               throw new java.io.IOException("INCORRECT format of the data");
               //System.out.println("<??>" + st.ttype );
            } 
         }
         line = "";
         for (int i=0;i<arrayPos;i++)
         {
           line += arrayDesc[i] + "#";
         }
         //System.out.println(line);
         
         //Comprobamos si las variables estan todas en la tabla de
         //atributos (las variables que no estén serán ignoradas)
         
         for (int i=0; i<numVars; i++)
         {
            if (!atrTab.containsKey(arrayDesc[i]))
            {
               System.out.println("WARNING: attribute table does NOT contain the attribute " + arrayDesc[i] + ". It will be skipped");
            }
         }

         
         //leer los datos
         arrayVal = new float[numVars];
         
         while(st.ttype != StreamTokenizer.TT_EOF)
         {  
            for (int i=0;i<numVars;i++)
            {
               if (st.ttype == StreamTokenizer.TT_NUMBER)
               {
                  arrayVal[i] = (float)st.nval;
                  //System.out.println("#" + arrayVal[i] + "#");        
               }   
               else 
               {
                  //System.out.println("<??>" + st.ttype );
                  throw new java.io.IOException("INCORRECT format of the data");
               }
               st.nextToken();
            }
            line = "";
            for (int i=0;i<numVars;i++)
            {
               line += "" + arrayVal[i] + "#";
            }
            //System.out.println(line);

            //enviar datos a Case.importFromData
			/* WARNING!!!! esto ya no funciona!!!
            auxCase = EToolsCase.importFromData(atrTab,
                                                dist,
                                                arrayDesc,
                                                arrayVal,
                                                numVars,
                                                in);      
			
                                                
            //System.out.println(auxCase); 
            caseVector.add(auxCase);
			*/
         }  
       }
       catch (IOException e)
       {
          e.printStackTrace(); 
       }
       
       return (caseVector);
            
   }   
   
}
