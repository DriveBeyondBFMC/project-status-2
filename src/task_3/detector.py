import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from typing import Literal

class Detector(object):
    def __init__(self, srcPoints: np.ndarray) -> None:

        self.srcPoints = srcPoints
    
        ratio = 1 / 3
        minX = srcPoints[:, 0].min()
        maxX = srcPoints[:, 0].max()
        minY = srcPoints[:, 1].min()    
        maxY = srcPoints[:, 1].max()
        self.perspectiveHeight = int(maxY - minY)
        self.perspectiveWidth = int((maxX - minX) * ratio)

        dstPoints = np.array([[0, 0], 
                              [self.perspectiveWidth, 0], 
                              [self.perspectiveWidth, self.perspectiveHeight], 
                              [0, self.perspectiveHeight]], dtype=np.float32)
        self.dstPoints = dstPoints

        self.warpMat = cv2.getPerspectiveTransform(srcPoints, dstPoints)
        self.invWarpMat = cv2.getPerspectiveTransform(dstPoints, srcPoints)

        lowerOffset = 52
        upperOffset = 72
        maskHeight = 35
        self.maskVehicleFrame = np.array([[lowerOffset, self.perspectiveHeight],
                                          [self.perspectiveWidth - lowerOffset, self.perspectiveHeight],
                                          [self.perspectiveWidth - upperOffset, self.perspectiveHeight - maskHeight],
                                          [upperOffset, self.perspectiveHeight - maskHeight]])

    def detection(self, image):
        warpImage = cv2.warpPerspective(image, self.warpMat, (self.perspectiveWidth, self.perspectiveHeight), flags = cv2.INTER_LANCZOS4)
        edgeImage = self.edgeDetector(warpImage)
        
        points = np.array(np.where(edgeImage == 255)).T

        angle = np.pi / 2
        edgeFrameWithPoints = np.zeros_like(edgeImage)
        if len(points):
            clusters, clustersPos = self.groupPoints(points, minDist = 10, minPoints = 30) # Filter out disjoint lane

            angle, edgeFrameWithPoints = self.midlaneFinder(clusters, clustersPos, 20, "mean")
        
        debugImage = edgeFrameWithPoints
        return angle * 180 / np.pi, debugImage

    @staticmethod
    def directionDraw(image, angle, length, position):
        dx = length * np.cos(angle) 
        dy = length * np.sin(angle)

        start = position
        end = (int(start[0] + dx), int(start[1] - dy))
        dim = len(image.shape)
        if dim == 3: cv2.line(image, start, end, (0, 255, 0), 2, cv2.LINE_AA)
        elif dim == 2: cv2.line(image, start, end, 255, 2, cv2.LINE_AA)

    @staticmethod
    def pointDrawer(points: np.ndarray, frame: np.ndarray) -> None:
        dim = len(frame.shape)
        for point in points:
            if dim == 3:
                cv2.circle(frame, (int(point[0]), int(point[1])), 3, (0, 0, 0), cv2.FILLED)
            elif dim == 2:
                cv2.circle(frame, (int(point[0]), int(point[1])), 3, 125, cv2.FILLED)

    def midlaneFinder(self, 
                      clusters: np.ndarray, 
                      clustersPosition: np.ndarray, 
                      partition: int,
                      mode: Literal['mean', 'closest', "furthest"] = 'mean') -> float:

        frameRight = np.zeros((self.perspectiveHeight, self.perspectiveWidth))
        frameLeft = frameRight.copy()
        frame = np.zeros((self.perspectiveHeight, self.perspectiveWidth))
        for points, pos in zip(clusters, clustersPosition):   
            if pos[0] > self.perspectiveWidth / 2:
                frameRight[points[:, 0], points[:, 1]] = 255
            else: frameLeft[points[:, 0], points[:, 1]] = 255
            frame[points[:, 0], points[:, 1]] = 255
            
        compressLeft = np.where(frameLeft.reshape(partition, -1, frameLeft.shape[1]).mean(1) > 0, 255, 0).astype(np.uint8)
        compressRight = np.where(frameRight.reshape(partition, -1, frameRight.shape[1]).mean(1) > 0, 255, 0).astype(np.uint8)

        deltaY = frameLeft.shape[0] / partition
        
        leftCompressionPoints = []
        rightCompressionPoints = []
        for idx, (rowLeft, rowRight) in enumerate(zip(compressLeft, compressRight)):
            leftPoints = np.array(np.where(rowLeft == 255)).T
            rightPoints = np.array(np.where(rowRight == 255)).T
            
            if len(leftPoints) == 0: leftCompressionPoints += [[np.inf, int(deltaY * (idx + .5))]]
            else:
                _, position = self.groupPoints(leftPoints, 1, 0)
                leftCompressionPoints += [[np.max(position), int(deltaY * (idx + .5))]]
            if len(rightPoints) == 0: rightCompressionPoints += [[np.inf, int(deltaY * (idx + .5))]]
            else:
                _, position = self.groupPoints(rightPoints, 1, 0)
                rightCompressionPoints += [[np.min(position), int(deltaY * (idx + .5))]]
                
        leftCompressionPoints = np.asarray(leftCompressionPoints)
        rightCompressionPoints = np.asarray(rightCompressionPoints)

        midPoints = (leftCompressionPoints + rightCompressionPoints) / 2
        midPoints = midPoints[np.isinf(midPoints.sum(1)) == False]
        infRightPos = np.isinf(rightCompressionPoints.sum(1))
        infLeftPos = np.isinf(leftCompressionPoints.sum(1))
        allInfRight = np.all(infRightPos)
        allInfLeft = np.all(infLeftPos)
        
        self.pointDrawer(clustersPosition, frame)       
        angle = np.pi / 2
        if len(midPoints) != 0:
            if mode == 'closest':
                choosenPoint = midPoints[-1]
            elif mode == 'furthest':
                choosenPoint = midPoints[0]
            elif mode == "mean":
                choosenPoint = midPoints.mean(0)
            else: raise ValueError("Only furthest, mean, closest] mode are allowed") 

            diff = choosenPoint - np.array([self.perspectiveWidth // 2, self.perspectiveHeight])
            angle = np.arctan2(-diff[1], diff[0])
            self.pointDrawer([choosenPoint], frame)
        
        elif allInfLeft == True and allInfRight == False:    
            rightCompressionPoints = rightCompressionPoints[~infRightPos]
            diff = rightCompressionPoints[:-1] - rightCompressionPoints[1:]
            if len(diff) > 0:
                angle = np.arctan2(-diff[:, 1], diff[:, 0]).mean()
                angles = np.arctan2(-diff[:, 1], diff[:, 0])
                angles = angles[np.isfinite(angles)]
                absAngle = np.abs(angles)
                angle = angles[np.argmax(absAngle[np.isfinite(absAngle)])]
                angle += 0.15
                
        elif allInfLeft == False and allInfRight == True:
            leftCompressionPoints = leftCompressionPoints[~infLeftPos]
            diff = leftCompressionPoints[:-1] - leftCompressionPoints[1:]
            if len(diff) > 0:
                angle = np.arctan2(-diff[:, 1], diff[:, 0]).mean()
                angles = np.arctan2(-diff[:, 1], diff[:, 0])
                angles = angles[np.isfinite(angles)]
                absAngle = np.abs(angles)
                angle = angles[np.argmax(absAngle[np.isfinite(absAngle)])]
                angle -= 0.15

        self.directionDraw(frame, angle, 30, np.array([self.perspectiveWidth // 2, self.perspectiveHeight]))
        return angle, frame
        

    @staticmethod        
    def groupPoints(points: np.ndarray, minDist = 1, minPoints: int = 0, maxPoints: int = np.inf):
        dbscan = DBSCAN(eps = minDist, min_samples = 1, algorithm = 'kd_tree').fit(points)
        labels = dbscan.labels_

        uniqueLabels = np.unique(labels)
        uniqueLabels = uniqueLabels[uniqueLabels != -1]
        clusters = []
        clustersPosition = []
        for label in uniqueLabels:
            mask = labels == label
            clusterPoints = points[mask]

            if minPoints <= len(clusterPoints) <= maxPoints:
                clusters.append(clusterPoints)
                clustersPosition.append(clusterPoints[:, ::-1].mean(axis = 0).astype(int))
                
        return clusters, clustersPosition

    def edgeDetector(self, image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurImage = cv2.GaussianBlur(grayImage, (5, 5), 3)

        _, thresh = cv2.threshold(blurImage, 110, 255, cv2.THRESH_BINARY)
        binary = np.where(np.abs(cv2.Scharr(thresh, cv2.CV_64F, 1, 0)) > 0, 255, 0).astype(np.uint8)

        # mask = cv2.fillPoly(np.zeros_like(binary), [self.maskVehicleFrame], 255) # Mask out the hood of vehicle
        # binary = np.where(mask == 255, 0, binary)
        return binary
