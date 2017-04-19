package com.company;

import lejos.hardware.lcd.LCD;
import sofia_kp.SSAP_sparql_response;
import sofia_kp.iKPIC_subscribeHandler2;
import wrapper.SmartSpaceException;
import wrapper.SmartSpaceKPI;
import wrapper.SmartSpaceTriplet;

import java.io.*;
import java.net.ConnectException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Objects;
import java.util.Vector;

/**
 * Created by user on 11.04.16.
 */
public class Retranslater implements iKPIC_subscribeHandler2 {
    //    Номер порта, по которому подключаться
    private final int outputPort = 5555;
    private final int port1 = 5555;
    private final int port2 = 6666;
    //    Хост основного робота, к которому подключаться
    private final String host = "192.168.1.100";
    //    Хост дополнительного робота, к которому подключаться
    private final String host2 = "192.168.1.127";
    //    Взаимодействие с SS.
    private SmartSpaceKPI smartSpaceKPI;
    //    Взаимодействие с основным роботом
    private DataOutputStream outputStream;
    //    Взаимодействие с дополнительным роботом
    private DataOutputStream outputStream2;

    //    Время ожидания ответа от другого робота
    private final long sleepTime = 10000;
    //    Условные обозначения для вставки и удаления из SS
    private final String removeCommand = "rem";
    private final String insertCommand = "ins";
    //    Команды для взаимодействия с роботами
    private final String readyCom = "ready";
    private final String distanceCom = "dist";
    private final String stopEvent = "stopped";
    //    Субъект, предикат и объект, используемые для подписки.
    private final String subscribeSubject = null;
    private final String subscribePredicate = "task";
    private final String subscribeObject = null;
    //    Названия объектов робота, заносимые в SS
//    private final String backMotorName = "backBlock";
//    private final String middleMotorName = "middleBlock";
//    private final String forwardMotorName = "frontBlock";
    private final String backMotorName = "block0";
    private final String middleMotorName = "block1";
    private final String forwardMotorName = "block2";
    private final String robotName = "robot1";
    private final String robot2Name = "robot2";
    //    private final String robotName = "robot";
    private final String moveEngineName = "moveEngine";
    private final String liftEngineName = "liftEngine";
    private final String robotPredicate = "hasPart";
    //    private final String robotPredicate = "has";
    private final String blockAmountPredicate = "blockAmount";
    private final String blockPredicate = "hasPart";
    //    private final String blockPredicate = "has";
    private final String blockAmount = "3";
    private final String blockAmount2 = "1";
    //    Функции робота, заносимые в SS
    private final String canPredicate = "implements";
    private final String goToFun = "goToLocation";
    private final String backFun = "moveBack";
    private final String forwardFun = "moveForward";
    private final String stopFun = "stop";
    private final String leftFun = "turnLeft";
    private final String rightFun = "turnRight";
    private final String acrossFun = "acrossObstacle";
    private final String exploreFun = "exploreLocation";
    private final String photoFun = "takePhoto";
    private final String obstacleFun = "exploreObstacle";
    private final String riseFun = "rise";
    private final String learnFun = "learn";

    private final String needPredicate = "needs";
    private final String locationPar = "coordinates";
    private final String obstacleInfoObject = "obstacleInfo";
    private final String robotLocationPredicate = "location";
    private final String acrossTypePredicate = "acrossType";
    //    Команды для робота/блоков.
    private final String commandForward = "moveForward";
    private final String commandBack = "moveBack";
    private final String commandLeft = "turnLeft";
    private final String commandRight = "turnRight";
    private final String commandTurn = "turnTurn";
    private final String commandStop = "stop";
    private final String commandExit = "exit";
    private final String commandShrink = "shrink";
    private final String commandLower = "lower";
    private final String commandRise = "rise";
    private final String commandClimb = "acrossObstacle";
    private final String commandExploreObst = "exploreObstacle";

    //    Константы для базы шаблонов
    private final String templateBDPredicate = "overcome";
    //    Минимальная значимая разница высоты препятствия
    private final int deltaH = 100;
    //    Минимальная значимая разница длины препятствия
    private final int deltaT = 1000;
    //    Минимальная значимая разница между высотами препятствий
    private final int minimalDifference = 700;

    //    Определяет, по какому адресу необходимо подключаться к SS: к роутеру или к компьютеру.
    boolean SSOnRouter;

    //    Сколько раз ещё нужно измерить препятствие
    private int timesToLearn = 0;

    //    Координаты роботов
    private int coorX1 = 0, coorY1 = 0, coorX2 = 0, coorY2 = -200;
    //    Направление второго робота
    private int curAngle = 0;
    //    Расстояние от препятствия, на котором следует изучать его
    private final int deltaX = 1, deltaY = -200;

    //    Конструктор
    Retranslater() {

    }

    /**
     * Подключиться к роботу, занести данные в SS.
     */
    void start() {
        boolean robo1 = true, robo2 = false;
        if (!robo1 && !robo2) {
//            analyseLearning2();
            String subject = "J|K";
            String[] split = subject.split("\\|");
//            accrossOld("93; 88; 103; 261; 261; 255; 264; 263; 257; 263; 264; 254; 263; 263; 253; 257; 263; 263; 253; 261; 261; 252; 260; 261; 262; 252; 261; 261; 252; 261; 261; 262; 141; 141; 144; 144; 131; 131; 142; 162; 154; 201; 206; 206; 206; 206; 206; 192; 125; 130; 262; 254; 257; 242; 142; 149; 179; 179; 166; 124; 135; 162; 155; 24; 24; 35");
//            accrossOld("12; 10; 22; 120; 120; 171; 182; 182; 182; 182; 182; 176; 89; 121; 121; 112; 87; 87; 100; 137; 133; 107; 107; 116; 137; 137; 126; 79; 83; 89; 89; 101; 136; 136; 130; 136; 261; 261; 256; 258; 246; 230; 232; 262; 262; 256; 251; 261; 261; 260; 228; 231; 264; 264; 255; 255; 263; 263; 253; 254; 261; 256; 263; 263; 254");
            return;
        }
        try {
//            Подключиться к роботам
            try {
                if (!robo1)
                    throw new ConnectException("Robot 1 is not included.");
                Socket socket = new Socket(host, outputPort);
                outputStream = new DataOutputStream(socket.getOutputStream());
                outputStream.writeBytes(String.format("%d\n", port1));
            } catch (ConnectException e) {
                System.out.println("Host " + host + " is not available.");
                e.printStackTrace();
                outputStream = null;
            }
            try {
                if (!robo2)
                    throw new ConnectException("Robot 2 is not included.");
                Socket socket2 = new Socket(host2, outputPort);
                outputStream2 = new DataOutputStream(socket2.getOutputStream());
                outputStream2.writeBytes(String.format("%d\n", port2));
            } catch (ConnectException e) {
                System.out.println("Host " + host2 + " is not available.");
                e.printStackTrace();
                outputStream2 = null;
            }
//            Занести данные в SS
            if (connectTry()) {
                Thread thread = new Thread(() -> waitForCommands(port1));
                thread.start();
                Thread thread2 = new Thread(() -> waitForCommands(port2));
                thread2.start();
//                Подписаться и ждать
                subscribe();
                System.out.println("I am ready!\n");
                System.in.read();
//                Выход
                if (outputStream != null)
                    try {
                        outputStream.writeBytes("0\t" + commandExit + "\n");
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                if (outputStream2 != null)
                    try {
                        outputStream2.writeBytes("0\t" + commandExit + "\n");
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                leave();
                Runtime.getRuntime().exit(0);
            } else {
                System.out.println("Cannot connect to SS.");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void analyseLearning2() {
//        Найти все виды препятствий
        connectTry();
        try {
            Vector<SmartSpaceTriplet> query = smartSpaceKPI.query(new SmartSpaceTriplet(null, null, null));
            ArrayList<String> obstacles = new ArrayList<>();
            String obstacleTypesStart = "obstacle_";
            for (SmartSpaceTriplet triplet :
                    query)
                if (triplet.getPredicate().startsWith(obstacleTypesStart))
                    if (!obstacles.contains(triplet.getPredicate()))
                        obstacles.add(triplet.getPredicate());
//            Проанализировать каждое
            for (String obstType :
                    obstacles) {
                Vector<SmartSpaceTriplet> chars = smartSpaceKPI.query(new SmartSpaceTriplet(null, obstType, null));
//                Пройтись по каждому, распарсить
                ArrayList<ArrayList<Integer>> heights = new ArrayList<>();
                for (SmartSpaceTriplet triplet :
                        chars) {
                    String[] split = triplet.getSubject().replace("test\t", "").split("; ");
                    ArrayList<Integer> curHeights = new ArrayList<>();
                    for (String height :
                            split) {
                        curHeights.add(safeParseInt(height));
                    }
                    heights.add(curHeights);
                }
//                Попарно сравнить
                ArrayList<Long> differences = new ArrayList<>();
                ArrayList<String> indexes = new ArrayList<>();
                long[] sums = new long[heights.size()];
                for (int i = 0; i < heights.size(); i++) {
                    for (int j = i + 1; j < heights.size(); j++) {
                        long obstacleDifference = 0;
                        for (int k = 0; k < heights.get(i).size(); k++) {
                            int diff = heights.get(i).get(k);
                            if (k < heights.get(j).size())
                                diff -= heights.get(j).get(k);
                            diff *= diff;
                            obstacleDifference += diff;
                        }
                        for (int k = heights.get(i).size(); k < heights.get(j).size(); k++)
                            obstacleDifference += Math.pow(heights.get(j).get(k), 2);
                        obstacleDifference = (long) Math.sqrt(obstacleDifference);
                        differences.add(obstacleDifference);
                        indexes.add(String.format("%d%d", i, j));
                        sums[i] += obstacleDifference;
                        sums[j] += obstacleDifference;
                    }
                }
//                Найти максимальную сумму
                int iMax = 0;
                for (int i = 1; i < sums.length; i++)
                    if (sums[i] > sums[iMax])
                        iMax = i;

//                Отсортировать и вывести разницу
//                differences.sort(null);
                String res = "Differences for " + obstType + ":\t\t";
                for (int i = 0; i < differences.size(); i++) {
                    if (!indexes.get(i).contains(String.format("%d", iMax)))
                        res += String.format("%s:\t%d\n", indexes.get(i), differences.get(i));
//                    res += String.format("%d, ", differences.get(i));
                }
                System.out.println(res);
            }

        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }
    }

    private void analyseLearning() {
//        Найти все виды препятствий
        connectTry();
        try {
            Vector<SmartSpaceTriplet> query = smartSpaceKPI.query(new SmartSpaceTriplet(null, null, null));
            ArrayList<String> obstacles = new ArrayList<>();
            String obstacleTypesStart = "obstacle_";
            for (SmartSpaceTriplet triplet :
                    query)
                if (triplet.getPredicate().startsWith(obstacleTypesStart))
                    if (!obstacles.contains(triplet.getPredicate()))
                        obstacles.add(triplet.getPredicate());
//            Проанализировать каждое
            for (String obstType :
                    obstacles) {
                Vector<SmartSpaceTriplet> chars = smartSpaceKPI.query(new SmartSpaceTriplet(null, obstType, null));
                ArrayList<Integer> minInts = new ArrayList<>(), maxInts = new ArrayList<>(),
                        mins = new ArrayList<>(), maxs = new ArrayList<>();
                int min = 0, max = Integer.MAX_VALUE;
                for (SmartSpaceTriplet triplet :
                        chars) {
                    String[] split = triplet.getObject().split(";");
                    for (int i = 0; i < split.length; i++) {
                        int curMin = safeParseInt(split[i].split("-")[0]), curMax = safeParseInt(split[i].split("-")[1]);
                        if (i >= mins.size()) {
                            mins.add(curMin);
                            maxs.add(curMax);
                        } else {
                            if (curMin < mins.get(i)) {
                                int tmp = curMin;
                                curMin = mins.get(i);
                                mins.set(i, tmp);
                            }
                            if (curMax > maxs.get(i)) {
                                int tmp = curMax;
                                curMax = maxs.get(i);
                                maxs.set(i, tmp);
                            }
                            if (i >= minInts.size()) {
                                minInts.add(curMin);
                                maxInts.add(curMax);
                            } else {
                                if (curMin < minInts.get(i))
                                    minInts.set(i, curMin);
                                if (curMax > maxInts.get(i))
                                    maxInts.set(i, curMax);
                            }
                        }
                    }
                }
                String res = "Intervals for " + obstType + ":\t\t";
                for (int i = 0; i < minInts.size(); i++)
                    res += String.format("%d-%d; ", minInts.get(i), maxInts.get(i));
                System.out.println(res);
            }

        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }
    }

    private String analyseObstacle(String obstacleHeights) {
        String[] heights = obstacleHeights.split("; ");
//        Отрезать первые и последние 10%
        float cut = 0.1f;
        int cutFirst = (int) (heights.length * cut), cutLast = (int) (heights.length * (1f - cut));
        for (int i = 0; i < cutFirst; i++)
            heights[i] = heights[cutFirst];
        for (int i = cutLast; i < heights.length; i++)
            heights[i] = heights[cutLast];

        String res = "";
//        Найти все перепады и проанализировать их
        for (int i = 0; i < heights.length; ) {
//            Копировать значения до очередного перепада
            ArrayList<Integer> curHeights = new ArrayList<>();
            int curValue = Integer.parseInt(heights[i]);
            do {
                curHeights.add(curValue);
                if (++i == heights.length)
                    break;
                curValue = Integer.parseInt(heights[i]);
            } while (curHeights.size() < 2 || Math.abs(curValue - curHeights.get(curHeights.size() - 2)) < 60);
//            Отсортировать значения и найти пороги
            curHeights.sort(null);
            res += String.format("%d-%d;", curHeights.get((int) Math.floor((double) curHeights.size() * 0.1)),
                    curHeights.get((int) Math.floor((double) curHeights.size() * 0.9)));
        }
//        Отсечь последний символ
        res = res.substring(0, res.length() - 1);
        return res;
    }

    /**
     * Ждать данные от робота, чтобы занести их в SS
     */
    private void waitForCommands(int port) {
        ServerSocket serverSocket;
        try {
//            Подключиться
            serverSocket = new ServerSocket(port);
            Socket socket = serverSocket.accept();
            BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            String[] commands;
            while (!serverSocket.isClosed()) {
//                Ждать данные
                String command = in.readLine();
                if (command == null)
                    continue;
//                Робот сообщил о готовности - известить об этом и ждать дальше
                if (command.equals(readyCom)) {
                    System.out.println((port == port1 ? robotName : robot2Name) + " is ready.");
                    continue;
                }
//                Робот сообщил о пройденной дистанции - изменить координаты и ждать дальше
                if (command.startsWith(distanceCom)) {
                    String[] distance = command.split("\t");
                    if (distance.length > 1) {
                        int dist = safeParseInt(distance[1]);
                        if (port == port1) {
                            coorX1 += dist;
                            smartSpaceKPI.remove(new SmartSpaceTriplet(robotName, robotLocationPredicate, null));
                            smartSpaceKPI.remove(new SmartSpaceTriplet(
                                    robotName, robotLocationPredicate, String.format("%d;%d", coorX1, coorY1)));
                        } else {
                            double angle = curAngle;
                            if (distance.length > 2)
                                angle = safeParseInt(distance[2]);
                            angle *= Math.PI / 180.0;
                            coorX2 += dist * Math.cos(angle);
                            coorY2 += dist * Math.sin(angle);
                            smartSpaceKPI.remove(new SmartSpaceTriplet(robot2Name, robotLocationPredicate, null));
                            smartSpaceKPI.insert(new SmartSpaceTriplet(
                                    robot2Name, robotLocationPredicate, String.format("%d;%d", coorX2, coorY2)));
                        }
                    }
                    continue;
                }
//                Робот сообщил о встреченном препятствии - запустить процесс преодоления препятствия
                if (command.startsWith(stopEvent)) {
                    String[] distance = command.split("\t");
                    if (distance.length > 1)
                        coorX1 += safeParseInt(distance[1]);
                    accrossOld();
                }
//                Обработка информации о высотах препятствия
                if (command.startsWith("test\t")) {
//                    smartSpaceKPI.remove(new SmartSpaceTriplet(null, obstacleInfoObject, null));
//                    smartSpaceKPI.insert(new SmartSpaceTriplet(command, obstacleInfoObject,
                    smartSpaceKPI.insert(new SmartSpaceTriplet(command, "obstacle_2-1",
                            analyseObstacle(command.split("\t")[1])));
                    outputStream2.writeBytes(String.format("0\t%s\t%d\n", commandBack, coorX2));
                    learning();
                }

                System.out.println(command);
//                Преобразовать в тройку и занести
                commands = command.split("\t");
                for (int i = 0; i < commands.length; i++)
                    if (commands[i].isEmpty())
                        commands[i] = null;
                if (commands.length >= 4)
                    if (commands[3].equals(insertCommand)) {
                        smartSpaceKPI.remove(new SmartSpaceTriplet(commands[0], commands[1], commands[2]));
                        smartSpaceKPI.insert(new SmartSpaceTriplet(commands[0], commands[1], commands[2]));
                    } else if (commands[3].equals(removeCommand))
                        smartSpaceKPI.remove(new SmartSpaceTriplet(commands[0], commands[1], commands[2]));
            }
        } catch (IOException | SmartSpaceException e) {
            e.printStackTrace();
        }
    }

    /**
     * Попытаться подключиться к SmartSpace.
     *
     * @return результат подключения: true, если без ошибок, или false, если произошло SmartSpaceException.
     */
    private boolean connectTry() {
        try {
            String routerHost = "192.168.1.1", PCHost = "192.168.2.101";
            smartSpaceKPI = new SmartSpaceKPI(SSOnRouter ? routerHost : PCHost, 10010, "x");
            if (outputStream != null) {
//        Занести в SmartSpace информацию об основном роботе и его блоках и моторах
                smartSpaceKPI.insert(new SmartSpaceTriplet(backMotorName, blockPredicate, moveEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(middleMotorName, blockPredicate, moveEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(forwardMotorName, blockPredicate, moveEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(forwardMotorName, blockPredicate, liftEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, robotPredicate, backMotorName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, robotPredicate, middleMotorName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, robotPredicate, forwardMotorName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, blockAmountPredicate, blockAmount));
//        Занести в SmartSpace информацию о функциях основного робота
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, goToFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, backFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, forwardFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, acrossFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, exploreFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, photoFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, canPredicate, obstacleInfoObject));
                smartSpaceKPI.insert(new SmartSpaceTriplet(exploreFun, needPredicate, locationPar));
                smartSpaceKPI.insert(new SmartSpaceTriplet(goToFun, needPredicate, locationPar));
            }
            if (outputStream2 != null) {
//        Занести в SmartSpace информацию о дополнительном роботе и его блоках и моторах
                smartSpaceKPI.insert(new SmartSpaceTriplet(middleMotorName, blockPredicate, moveEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(middleMotorName, blockPredicate, liftEngineName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, robotPredicate, middleMotorName));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, blockAmountPredicate, blockAmount2));
//        Занести в SmartSpace информацию о функциях дополнительного робота
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, goToFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, backFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, forwardFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, leftFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, rightFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(robot2Name, canPredicate, obstacleFun));
                smartSpaceKPI.insert(new SmartSpaceTriplet(obstacleFun, needPredicate, locationPar));
                smartSpaceKPI.insert(new SmartSpaceTriplet(goToFun, needPredicate, locationPar));
            }
        } catch (SmartSpaceException e) {
            return false;
        }
        return true;
    }

    /**
     * Подписаться на приём команд
     */
    private void subscribe() {
        try {
            smartSpaceKPI.subscribe(new SmartSpaceTriplet(subscribeSubject, subscribePredicate, subscribeObject), this);
        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }
    }

    /**
     * Безопасный выход из SmartSpace. Необходимо вызывать при завершении работы со SmartSpace.
     */
    private void leave() {
        if (smartSpaceKPI != null)
            try {
                smartSpaceKPI.remove(new SmartSpaceTriplet(robotName, null, null));
                smartSpaceKPI.remove(new SmartSpaceTriplet(robot2Name, null, null));
                smartSpaceKPI.remove(new SmartSpaceTriplet(null, blockPredicate, null));
                smartSpaceKPI.remove(new SmartSpaceTriplet(null, needPredicate, null));
                smartSpaceKPI.remove(new SmartSpaceTriplet(null, "event", null));
                smartSpaceKPI.leave();
            } catch (SmartSpaceException e) {
                e.printStackTrace();
            }
    }

    private void accrossOld() {
        try {
//            Отъехать ненамного
            outputStream.writeBytes(String.format("%d\t%s\n", 0, commandBack + "\t" + "200"));

//                        отправить задачу с координатами для исследования
            smartSpaceKPI.remove(new SmartSpaceTriplet(obstacleFun, locationPar, null));
            smartSpaceKPI.insert(new SmartSpaceTriplet(obstacleFun, locationPar, String.format("%d; %d", coorX1, coorY1)));
            smartSpaceKPI.remove(new SmartSpaceTriplet(null, subscribePredicate, obstacleFun));
            smartSpaceKPI.insert(new SmartSpaceTriplet(null, subscribePredicate, obstacleFun));
        } catch (SmartSpaceException | IOException e) {
            e.printStackTrace();
        }
    }

    String accrossOld(String obstacleObject) {
        try {
//            Парсить параметры
            String[] obstacleSplit = obstacleObject.split("; ");
            int[] obstacleHeights = new int[obstacleSplit.length];
            for (int i = 0; i < obstacleSplit.length; i++) {
                if (i == 0) {
                    obstacleHeights[i] = Integer.parseInt(obstacleSplit[i]);
                } else {
                    obstacleHeights[i] = Integer.parseInt(obstacleSplit[i].split(": ")[0]);
                }
            }
//            Проверить базу шаблонов
            Vector<SmartSpaceTriplet> templateBD = smartSpaceKPI.query(
                    new SmartSpaceTriplet(null, templateBDPredicate, null));
            int templateIndex = -1;
            long minDiff = Long.MAX_VALUE;
            for (int i = 0; i < templateBD.size(); i++) {
                obstacleSplit = templateBD.get(i).getSubject().split("; ");
                long obstacleDifference = 0;
                for (int j = 0; j < obstacleSplit.length; j++) {
                    int diff = safeParseInt(obstacleSplit[j]);
                    if (j < obstacleHeights.length)
                        diff -= obstacleHeights[j];
                    diff *= diff;
                    obstacleDifference += diff;
                }
                for (int j = obstacleSplit.length; j < obstacleHeights.length; j++)
                    obstacleDifference += obstacleHeights[j];
                obstacleDifference = (long) Math.sqrt(obstacleDifference);
                if (obstacleDifference < minDiff) {
                    templateIndex = i;
                    minDiff = obstacleDifference;
                }
            }
//            Если шаблон найден, сообщить об этом
            if (minDiff <= minimalDifference) {
                System.out.println(String.format("The closest obstacle (difference = \t%d\t):\n%s",
                        minDiff, templateBD.get(templateIndex).getObject()));
                return templateBD.get(templateIndex).getObject();
//                TODO: применить шаблон
            } else {
                System.out.println("Unknown obstacle:\n" + obstacleObject);
                smartSpaceKPI.insert(new SmartSpaceTriplet(obstacleObject, templateBDPredicate,
                        String.format("commands for %d-%d", obstacleHeights[0], obstacleHeights[obstacleHeights.length - 1])));
//                TODO: если такого препятствия нет, то передать управление
            }

        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }
        return "";
    }

    private void accross() {
        Vector<SmartSpaceTriplet> obstaclePars = new Vector<>();
        try {
//            Отъехать ненамного
            outputStream.writeBytes(String.format("%d\t%s\n", 0, commandBack + "\t" + "200"));

//                        отправить задачу с координатами для исследования
            smartSpaceKPI.remove(new SmartSpaceTriplet(obstacleFun, locationPar, null));
            smartSpaceKPI.insert(new SmartSpaceTriplet(obstacleFun, locationPar, String.format("%d; %d", coorX1, coorY1)));
            smartSpaceKPI.remove(new SmartSpaceTriplet(null, subscribePredicate, obstacleFun));
            smartSpaceKPI.insert(new SmartSpaceTriplet(null, subscribePredicate, obstacleFun));
//                        ждать ответ
            while (obstaclePars.size() == 0) {
                Thread.sleep(sleepTime);
                obstaclePars = smartSpaceKPI.query(new SmartSpaceTriplet(null, obstacleInfoObject, null));
            }
            smartSpaceKPI.remove(obstaclePars.firstElement());
            String obstacleObject = obstaclePars.firstElement().getObject();
            System.out.println(String.format("Obstacle info: %s", obstacleObject));
            // TODO: обработать инфу
        } catch (SmartSpaceException | InterruptedException | IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Распознать посланную команду и переслать её роботу. Распознаваемые команды:
     * [blockName] commandIs forward: двигаться вперёд, пока не придёт stop.
     * [blockName] commandIs back: двигаться назад, пока не придёт stop.
     * [blockName] commandIs stop: остановиться.
     * (Если вместо [blockName] указать robot, то команда будет подана на все модули.)
     * block2 commandIs rise: поднять передний модуль.
     * block2 commandIs lower: опустить передний модуль.
     * block2 commandIs shrink: резко опустить передний модуль, чтобы подтянуть остальные.
     * robot commandIs across: преодолеть препятствие.
     * robot commandIs exit: завершить выполнение.
     */
    @Override
    public void kpic_RDFEventHandler(Vector<Vector<String>> vector, Vector<Vector<String>> vector1, String s, String s1) {
        for (Vector<String> tripletString : vector) {
            SmartSpaceTriplet triplet = new SmartSpaceTriplet(tripletString);
//            Определить исполнителя
            String execRobot = "", execBlock = "";
            String task = triplet.getObject(), subject = triplet.getSubject();
//            Если исполнитель задан явно, то отправить ему, иначе искать того, кто может
            boolean isMainRobot = true;
            String[] split = subject.split("\\|");
            subject = split[0];
            if (split.length > 1)
                execBlock = split[1];

            switch (subject) {
                case robotName:
                    execRobot = host;
                    break;
                case robot2Name:
                    execRobot = host2;
                    isMainRobot = false;
                    break;
                default:
                    try {
                        Vector<SmartSpaceTriplet> executors = smartSpaceKPI.query(new SmartSpaceTriplet(null, canPredicate, task));
                        for (SmartSpaceTriplet executor :
                                executors) {
                            switch (executor.getSubject()) {
                                case robotName:
                                    execRobot = host;
                                    break;
                                case robot2Name:
                                    execRobot = host2;
                                    isMainRobot = false;
                                    break;
                                default:
                                    continue;
                            }
                            break;
                        }
                    } catch (SmartSpaceException e) {
                        e.printStackTrace();
                    }
                    break;
            }
//            Если исполнитель не определён, то перейти к следующей команде
            if (execRobot.isEmpty()) {
                System.out.println(String.format("Couldn't find an executor for triplet <\"%s\", \"%s\", \"%s\">",
                        triplet.getSubject(), triplet.getPredicate(), triplet.getObject()));
                continue;
            }
            System.out.println(String.format("Triplet: <\"%s\", \"%s\", \"%s\">\nExecutor's host: %s",
                    triplet.getSubject(), triplet.getPredicate(), triplet.getObject(), execRobot));

//            Анализировать триплет, в соответствии с этим передавать конкретные команды
            ArrayList<String> commands = new ArrayList<>();
            ArrayList<Long> delays = new ArrayList<>();
            ArrayList<String> goToCommand;
            switch (task) {
                case backFun:
                    delays.add(0L);
                    commands.add(commandBack);
                    break;
                case forwardFun:
                    delays.add(0L);
                    commands.add(commandForward);
                    break;
                case leftFun:
                    delays.add(0L);
                    commands.add(commandLeft);
                    curAngle += 270;
                    curAngle %= 360;
                    break;
                case rightFun:
                    delays.add(0L);
                    commands.add(commandRight);
                    curAngle += 90;
                    curAngle %= 360;
                    break;

                case acrossFun:
//                    Thread acrossThread = new Thread(() -> accrossOld(true));
//                    acrossThread.start();
                    String acrossType = "";
                    try {
                        Vector<SmartSpaceTriplet> query = smartSpaceKPI.query(
                                new SmartSpaceTriplet(acrossFun, acrossTypePredicate, null));
                        if (!query.isEmpty()) {
                            acrossType = query.get(0).getObject();
                            smartSpaceKPI.remove(query.firstElement());
                        }
                        if (acrossType == null)
                            acrossType = "";
                    } catch (SmartSpaceException e) {
                        e.printStackTrace();
                    }
                    if (acrossType.isEmpty())
                        System.out.println("I do not know how to climb it.");
                    else {
                        delays.add(0L);
                        commands.add(commandClimb + "\t" + acrossType);
                    }
                    break;

                case goToFun:
                    goToCommand = getGoToCommand(task, isMainRobot, false);
                    for (String gtCom : goToCommand) {
                        commands.add(gtCom);
                        delays.add(0L);
                    }
                    break;
                case exploreFun:
                    goToCommand = getGoToCommand(task, isMainRobot, false);
                    for (String gtCom : goToCommand) {
                        commands.add(gtCom);
                        delays.add(0L);
                    }
                    System.out.println("Функция фото пока не поддерживается.");
                    break;
                case photoFun:
                    System.out.println("Функция фото пока не поддерживается.");
                    break;
                case obstacleFun:
//                    Подъехать и повернуться
                    goToCommand = getGoToCommand(task, isMainRobot, true);
                    for (String gtCom : goToCommand) {
                        commands.add(gtCom);
                        delays.add(0L);
                    }
                    /*int angle, turnAngle;
                    if (coorY2 > coorY1)
                        angle = -90;
                    else angle = 90;
                    turnAngle = angle - curAngle;
                    if (turnAngle < -180)
                        turnAngle += 360;
                    else if (turnAngle > 180)
                        turnAngle -= 360;
                    delays.add(0L);
                    commands.add(String.format("%s\t%d", commandTurn, turnAngle));
                    curAngle = angle;*/
//                замерить
                    delays.add(0L);
                    commands.add(commandExploreObst);
                    break;
                case stopFun:
                    delays.add(0L);
                    commands.add(commandStop);
                    break;
                case riseFun:
                    delays.add(0L);
                    commands.add(commandRise);
                    break;
                case "lower":
                    delays.add(0L);
                    commands.add("lower");
                    break;
                case "shrink":
                    delays.add(0L);
                    commands.add("shrink");
                    break;
                case learnFun:
                    timesToLearn = 3;
                    learning();
                    break;
                case obstacleInfoObject:
                    String obstType = accrossOld(triplet.getSubject());
                    try {
                        smartSpaceKPI.insert(new SmartSpaceTriplet(acrossFun, acrossTypePredicate, obstType));
                        smartSpaceKPI.insert(new SmartSpaceTriplet(subject, subscribePredicate, acrossFun));
                    } catch (SmartSpaceException e) {
                        e.printStackTrace();
                    }
                    break;
            }

//            Отправить команды
            String allComs = "";
            for (int i = 0; i < commands.size(); i++)
                if (!commands.get(i).isEmpty())
                    allComs += String.format("%d\t%s\t%s\n", delays.get(i), execBlock, commands.get(i));
            if (!allComs.isEmpty())
                try {
                    if (outputStream != null && Objects.equals(execRobot, host))
                        outputStream.writeBytes(allComs);
                    else if (outputStream2 != null && Objects.equals(execRobot, host2))
                        outputStream2.writeBytes(allComs);
                } catch (IOException e) {
                    e.printStackTrace();
                }

//            Удалить триплет
            try {
                smartSpaceKPI.remove(triplet);
            } catch (SmartSpaceException e) {
                e.printStackTrace();
            }
        }
    }

    private void learning() {
        if (timesToLearn-- > 0)
            try {
                outputStream2.writeBytes(String.format("0\t%s\n", commandExploreObst));
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    private ArrayList<String> getGoToCommand(String task, boolean isMainRobot, boolean nearby) {
        ArrayList<String> result = new ArrayList<>();
        try {
            Vector<SmartSpaceTriplet> coordinates = smartSpaceKPI.query(new SmartSpaceTriplet(task, locationPar, null));
            smartSpaceKPI.remove(new SmartSpaceTriplet(task, locationPar, null));
            if (coordinates != null && coordinates.size() > 0) {
                String coordinate = coordinates.get(0).getObject();
                String[] coors = coordinate.split("; ");
                if (coors.length > 1) {
//                преобразовать координаты в расстояние
                    int[] XY = {0, 0};
                    XY[0] = safeParseInt(coors[0]) - (isMainRobot ? coorX1 : coorX2);
                    if (!isMainRobot) {
                        XY[1] = safeParseInt(coors[1]) - coorY2;
//                    Если нужно подъехать рядом, то скорректировать расстояние
                        if (nearby) {
                            if (XY[0] >= 0)
                                XY[0] += deltaX;
                            else
                                XY[0] -= deltaX;
                            if (XY[1] > 0)
                                XY[1] += deltaY;
                            else
                                XY[1] -= deltaY;
                        }
//                    Повернуться на нужный угол
                        int angle;
                        if (XY[0] == 0)
                            angle = XY[1] > 0 ? 90 : -90;
                        else
                            angle = (int) (Math.atan((double) XY[1] / XY[0]) * 180.0 / Math.PI);
                        if (XY[0] < 0)
                            angle += 180;
                        if (angle != curAngle) {
                            result.add(String.format("%s\t%d", commandTurn, angle - curAngle));
                            curAngle = angle;
                        }
//                        Вычислить гипотенузу
                        XY[0] = (int) Math.sqrt(Math.pow(XY[0], 2) + Math.pow(XY[1], 2));
                    }
//                    Если ехать никуда не надо, то не отправлять команду
                    if (XY[0] == 0 && XY[1] == 0)
                        return result;
                    coordinate = String.format("%d", XY[0]);

                }
                String command = commandForward + "\t" + coordinate;
                if (!isMainRobot)
                    command += String.format("\t%d", curAngle);
                result.add(command);
                return result;
            } else {
                System.out.println("Не заданы координаты для задачи " + task + ".");
            }
        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }
        return result;
    }

    /**
     * Получить число из строки, не получая исключений.
     *
     * @param s строка, в которой число
     * @return число, которое было в строке, либо 0, если распарсить не удалось.
     */
    private int safeParseInt(String s) {
        int value;
        try {
            value = new Integer(s);
        } catch (NumberFormatException e) {
            value = 0;
        }
        return value;
    }

    @Override
    public void kpic_SPARQLEventHandler(SSAP_sparql_response ssap_sparql_response, SSAP_sparql_response ssap_sparql_response1, String s, String s1) {

    }

    @Override
    public void kpic_UnsubscribeEventHandler(String s) {

    }

    @Override
    public void kpic_ExceptionEventHandler(Throwable throwable) {

    }
}
