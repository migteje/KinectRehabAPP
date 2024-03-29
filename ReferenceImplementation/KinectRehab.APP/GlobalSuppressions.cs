//------------------------------------------------------------------------------
// <copyright file="GlobalSuppressions.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// This file is used by Code Analysis to maintain SuppressMessage 
// attributes that are applied to this project.
// Project-level suppressions either have no target or are given 
// a specific target and scoped to a namespace, type, member, etc.
//
// To add a suppression to this file, right-click the message in the 
// Code Analysis results, point to "Suppress Message", and click 
// "In Suppression File".
// You do not need to add suppressions to this file manually.

[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1045:DoNotPassTypesByReference", MessageId = "0#", Scope = "member", Target = "KinectRehab.APP.Common.BindableBase.#SetProperty`1(!!0&,!!0,System.String)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1026:DefaultParametersShouldNotBeUsed", Scope = "member", Target = "KinectRehab.APP.Common.BindableBase.#SetProperty`1(!!0&,!!0,System.String)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1026:DefaultParametersShouldNotBeUsed", Scope = "member", Target = "KinectRehab.APP.Common.BindableBase.#OnPropertyChanged(System.String)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1062:Validate arguments of public methods", MessageId = "0", Scope = "member", Target = "KinectRehab.APP.ItemDetailPage.#SaveState(System.Collections.Generic.Dictionary`2<System.String,System.Object>)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Security", "CA2104:DoNotDeclareReadOnlyMutableReferenceTypes", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage.#DefaultViewModelProperty")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA2227:CollectionPropertiesShouldBeReadOnly", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage.#DefaultViewModel")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "e", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage.#GoHome(System.Object,Windows.UI.Xaml.RoutedEventArgs)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "e", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage.#GoBack(System.Object,Windows.UI.Xaml.RoutedEventArgs)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1062:Validate arguments of public methods", MessageId = "0", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage.#OnNavigatedTo(Windows.UI.Xaml.Navigation.NavigationEventArgs)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1062:Validate arguments of public methods", MessageId = "0", Scope = "member", Target = "KinectRehab.APP.Common.LayoutAwarePage+ObservableDictionary`2.#CopyTo(System.Collections.Generic.KeyValuePair`2<!0,!1>[],System.Int32)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Security", "CA2104:DoNotDeclareReadOnlyMutableReferenceTypes", Scope = "member", Target = "KinectRehab.APP.Common.RichTextColumns.#RichTextContentProperty")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Security", "CA2104:DoNotDeclareReadOnlyMutableReferenceTypes", Scope = "member", Target = "KinectRehab.APP.Common.RichTextColumns.#ColumnTemplateProperty")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1012:AbstractTypesShouldNotHaveConstructors", Scope = "type", Target = "KinectRehab.APP.Data.SampleDataCommon")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA2204:Literals should be spelled correctly", MessageId = "SuspensionManager", Scope = "member", Target = "KinectRehab.APP.Common.SuspensionManagerException.#.ctor(System.Exception)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1014:MarkAssembliesWithClsCompliant")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA2210:AssembliesShouldHaveValidStrongNames")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA2204:Literals should be spelled correctly", MessageId = "HandReachExtent", Scope = "member", Target = "KinectRehab.APP.KinectPointerPointSample.#RenderPointer(System.Boolean,Windows.Foundation.Point,Windows.Foundation.Point,System.Single,System.TimeSpan,System.UInt64,WindowsPreview.Kinect.Input.HandType)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "Windows.UI.Xaml.Controls.TextBlock.put_Text(System.String)", Scope = "member", Target = "KinectRehab.APP.KinectPointerPointSample.#RenderPointer(System.Boolean,Windows.Foundation.Point,Windows.Foundation.Point,System.Single,System.TimeSpan,System.UInt64,WindowsPreview.Kinect.Input.HandType)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA2204:Literals should be spelled correctly", MessageId = "BodyTrackingId", Scope = "member", Target = "KinectRehab.APP.KinectPointerPointSample.#RenderPointer(System.Boolean,Windows.Foundation.Point,Windows.Foundation.Point,System.Single,System.TimeSpan,System.UInt64,WindowsPreview.Kinect.Input.HandType)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA2204:Literals should be spelled correctly", MessageId = "HandType", Scope = "member", Target = "KinectRehab.APP.KinectPointerPointSample.#RenderPointer(System.Boolean,Windows.Foundation.Point,Windows.Foundation.Point,System.Single,System.TimeSpan,System.UInt64,WindowsPreview.Kinect.Input.HandType)")]
[assembly: System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA2204:Literals should be spelled correctly", MessageId = "BodyTimeCounter", Scope = "member", Target = "KinectRehab.APP.KinectPointerPointSample.#RenderPointer(System.Boolean,Windows.Foundation.Point,Windows.Foundation.Point,System.Single,System.TimeSpan,System.UInt64,WindowsPreview.Kinect.Input.HandType)")]
